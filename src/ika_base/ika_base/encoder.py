"""Kadratür enkoder çözme (saf mantık) ve libgpiod olay okuyucu.

İki mod:
  x4 - A ve B kanallarının her iki kenarı da olay olarak dinlenir; seviyeler
       olaylardan takip edilir, işlem gecikse bile sayım doğru kalır.
       Çözünürlük: ppr * 4.
  x2 - Yalnızca A kanalının kenarları dinlenir, B olay İŞLENİRKEN okunur.
       Olay sayısı yarıya iner ama B'nin okunduğu an olaydan geç olduğu için
       yalnızca düşük PPR / düşük devirde güvenilirdir (546 PPR'lik
       DCGM-3865 için x4 kullanın). Çözünürlük: ppr * 2.

Yön kuralı: ileri dönüşte A, B'den önce gelir (00 -> 10 -> 11 -> 01).
Tersi ölçülüyorsa ilgili enkoder için invert: true verin.

Gerçek donanımda okuyucu ayrı bir süreçte çalışır (ProcessEncoderReader):
aynı süreçteki ROS döngüsüyle GIL'i paylaşan okuma thread'i yükte geri
kalıyor, kernel olay tamponu (1024) taşıyor ve x4 sayımı çöp oluyordu.
"""

import ctypes
import multiprocessing
import os
import select
import signal
import threading
import time

# (önceki durum, yeni durum) -> +1; durum = (A << 1) | B
_FORWARD = {(0b00, 0b10), (0b10, 0b11), (0b11, 0b01), (0b01, 0b00)}
_BACKWARD = {(new, old) for old, new in _FORWARD}

MODES = ('x4', 'x2', 'none')


def multiplier(mode: str) -> int:
    return {'x4': 4, 'x2': 2}.get(mode, 0)


class QuadratureDecoder:

    def __init__(self, mode: str = 'x4', invert: bool = False, a: int = 0, b: int = 0):
        if mode not in ('x4', 'x2'):
            raise ValueError(f"enkoder modu x4 veya x2 olmalı, gelen: {mode!r}")
        self.mode = mode
        self.invert = invert
        self.a = a
        self.b = b
        self.count = 0
        self.errors = 0     # x4'te kaçırılmış kenar sayısı

    def on_edge(self, channel: str, level: int, other_level: int = None) -> int:
        """channel: 'a' veya 'b'. x2 modunda other_level, A kenarı anındaki
        okunmuş B seviyesidir."""
        prev = (self.a << 1) | self.b
        if channel == 'a':
            self.a = level
            if other_level is not None:
                self.b = other_level
        else:
            self.b = level

        if self.mode == 'x2':
            delta = 1 if self.a != self.b else -1
        else:
            curr = (self.a << 1) | self.b
            if (prev, curr) in _FORWARD:
                delta = 1
            elif (prev, curr) in _BACKWARD:
                delta = -1
            else:
                # Seviye değişmeden kenar geldi: arada bir olay kaçırılmış
                # (kernel olay tamponu taştı ya da gürültü).
                self.errors += 1
                delta = 0

        if self.invert:
            delta = -delta
        self.count += delta
        return delta


class EncoderReader:
    """Bir veya daha fazla enkoderi arka plan thread'lerinde okur.

    encoders: [(a_pin, b_pin, invert), ...]; pin = (chip_adı, line_offset).
    Hatlar farklı GPIO chip'lerindeyse olaylar ayrı kuyruklardan gelir; tek
    thread hepsini okuyup kernel zaman damgasına göre sıralayarak işler.
    """

    def __init__(self, open_chip, encoders, mode: str = 'x4', bias: str = 'as-is'):
        self._mode = mode
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads = []
        self._requests = []
        self.decoders = []

        # chip -> {offset: (decoder index, kanal)}
        routes = {}
        for idx, (a_pin, b_pin, invert) in enumerate(encoders):
            self.decoders.append(QuadratureDecoder(mode, invert))
            routes.setdefault(a_pin[0], {})[a_pin[1]] = (idx, 'a')
            if mode == 'x4':
                routes.setdefault(b_pin[0], {})[b_pin[1]] = (idx, 'b')
        self._b_pins = [b for _, b, _ in encoders]

        self._b_inputs = {}
        if mode == 'x2':
            by_chip = {}
            for chip_name, offset in self._b_pins:
                by_chip.setdefault(chip_name, []).append(offset)
            for chip_name, offsets in by_chip.items():
                req = open_chip(chip_name).request_inputs(offsets, bias=bias)
                self._requests.append(req)
                for offset in offsets:
                    self._b_inputs[(chip_name, offset)] = req

        level_sources = {}
        sources = []
        for chip_name, route in routes.items():
            req = open_chip(chip_name).request_edges(list(route), bias=bias)
            self._requests.append(req)
            sources.append((req, route))
            for offset in route:
                level_sources[(chip_name, offset)] = req
        self._threads.append(threading.Thread(target=self._run, args=(sources,), daemon=True))

        for idx, (a_pin, b_pin, _) in enumerate(encoders):
            dec = self.decoders[idx]
            dec.a = level_sources[a_pin].get(a_pin[1])
            b_source = level_sources.get(b_pin) or self._b_inputs[b_pin]
            dec.b = b_source.get(b_pin[1])

        for thread in self._threads:
            thread.start()

    def _run(self, sources):
        if len(sources) == 1:
            req, route = sources[0]
            while not self._stop.is_set():
                if req.wait(0.1):
                    self._apply([(edge, route) for edge in req.read()])
            return

        # Birden fazla kuyruk: tarama başındaki andan önceki olayların hepsi
        # bu taramada okunmuş olur; yalnızca onlar sıralanıp işlenir, sonrakiler
        # bir sonraki taramaya kalır. Olaylar CLOCK_MONOTONIC ile damgalanır;
        # saat uyuşmuyorsa (eski kernel) olay beklemeden işlenir.
        # libgpiod v2 kuyrukları fd verir: select ile olay gelince uyanılır.
        # fd yoksa (v1, mock) kuyruklar kısa aralıklarla yoklanır.
        fds = [req.fileno() for req, _ in sources]
        waitable = all(fd is not None for fd in fds)
        pending = []
        while not self._stop.is_set():
            if waitable:
                # Geç kalan olay varsa hemen yeniden kes; yoksa olay gelene dek uyu.
                ready, _, _ = select.select(fds, [], [], 0.001 if pending else 0.1)
                if not ready and not pending:
                    continue
            cutoff = time.monotonic_ns()
            for req, route in sources:
                if req.wait(0.0 if waitable else 0.001):
                    pending.extend((edge, route) for edge in req.read())
            if not pending:
                continue
            pending.sort(key=lambda item: item[0].timestamp_ns)
            late = [cutoff < edge.timestamp_ns <= cutoff + 1_000_000_000 for edge, _ in pending]
            ready_items = [item for item, wait in zip(pending, late) if not wait]
            pending = [item for item, wait in zip(pending, late) if wait]
            if ready_items:
                self._apply(ready_items)

    def _apply(self, items):
        with self._lock:
            for edge, route in items:
                idx, channel = route[edge.offset]
                other = None
                if self._mode == 'x2':
                    b_pin = self._b_pins[idx]
                    other = self._b_inputs[b_pin].get(b_pin[1])
                self.decoders[idx].on_edge(channel, edge.level, other)

    def counts(self) -> list:
        with self._lock:
            return [d.count for d in self.decoders]

    def errors(self) -> list:
        with self._lock:
            return [d.errors for d in self.decoders]

    def close(self):
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=1.0)
        for req in self._requests:
            req.release()


# Alt süreç her bu kadar saniyede sayımları paylaşılan belleğe yazar.
_PUBLISH_PERIOD = 0.002
_START_TIMEOUT = 20.0
_PR_SET_PDEATHSIG = 1


def _reader_main(open_chip, encoders, mode, bias, rt_priority, shared, conn, stop):
    """ProcessEncoderReader'ın alt süreci: EncoderReader'ı çalıştırır,
    sayım + hata değerlerini `shared`'a ([sayımlar..., hatalar...]) yazar."""
    parent = os.getppid()
    try:
        # Ana süreç (ika_base_node) ölürse bu süreç de ölsün; yoksa GPIO
        # hatlarını tutup servisin yeniden başlamasını engeller.
        ctypes.CDLL(None, use_errno=True).prctl(_PR_SET_PDEATHSIG, signal.SIGTERM)
    except (OSError, AttributeError):
        pass
    # Sinyal işleyicide stop.set() çağrılmaz: Event'in kilidi o an tutuluyor
    # olabilir; düz bir bayrak yeterli.
    terminated = []
    signal.signal(signal.SIGTERM, lambda *_: terminated.append(True))
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    note = None
    if rt_priority > 0:
        # Okuma thread'i bu ayarı devralır: başka süreçler onu bekletemez.
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(rt_priority))
        except (OSError, AttributeError) as e:
            note = f'enkoder okuyucuya gerçek zamanlı öncelik verilemedi ({e})'
    try:
        reader = EncoderReader(open_chip, encoders, mode=mode, bias=bias)
    except Exception as e:  # noqa: BLE001 - hatayı ana sürece taşı
        conn.send(('error', f'{type(e).__name__}: {e}'))
        return
    n = len(encoders)
    try:
        conn.send(('ok', note))
        while not terminated and not stop.is_set() and os.getppid() == parent:
            shared[:n] = reader.counts()
            shared[n:] = reader.errors()
            time.sleep(_PUBLISH_PERIOD)
    finally:
        reader.close()


class ProcessEncoderReader:
    """EncoderReader ile aynı arayüz; okuma ve çözme ayrı bir süreçte çalışır.

    open_chip pickle edilebilir olmalı, ör.
    functools.partial(ika_hal.gpio.open_chip, backend='gpiod'): alt süreç
    'spawn' ile temiz başlar ve chip'leri kendisi açar. Sayımlar en fazla
    birkaç ms gecikmeyle okunur. Paylaşılan dizi kilitsizdir: alt süreç
    kilidi tutarken ölürse kontrol döngüsü kilitlenmesin (64 bit tek değer
    okuma/yazma aarch64'te bölünmez; sol/sağ arası ms'lik fark önemsiz).
    """

    def __init__(self, open_chip, encoders, mode: str = 'x4', bias: str = 'as-is',
                 rt_priority: int = 0):
        ctx = multiprocessing.get_context('spawn')
        self._n = len(encoders)
        self._shared = ctx.RawArray('q', 2 * self._n)
        self._stop = ctx.Event()
        parent_conn, child_conn = ctx.Pipe(duplex=False)
        self._proc = ctx.Process(
            target=_reader_main, name='ika_encoder', daemon=True,
            args=(open_chip, list(encoders), mode, bias, rt_priority,
                  self._shared, child_conn, self._stop))
        self._proc.start()
        child_conn.close()
        self.note = None
        if not parent_conn.poll(_START_TIMEOUT):
            self.close()
            raise RuntimeError(f'enkoder süreci {_START_TIMEOUT:.0f} s içinde başlamadı')
        try:
            status, detail = parent_conn.recv()
        except EOFError:
            status, detail = 'error', f'enkoder süreci çıktı (kod {self._proc.exitcode})'
        if status != 'ok':
            self.close()
            raise RuntimeError(f'enkoder okuyucu açılamadı: {detail}')
        self.note = detail
        # İlk değerler yazılana kadar bekle (counts() başlangıçta 0 döner).
        time.sleep(4 * _PUBLISH_PERIOD)

    def alive(self) -> bool:
        return self._proc.is_alive()

    def counts(self) -> list:
        return list(self._shared[:self._n])

    def errors(self) -> list:
        return list(self._shared[self._n:])

    def close(self):
        self._stop.set()
        self._proc.join(timeout=2.0)
        if self._proc.is_alive():
            self._proc.terminate()
            self._proc.join(timeout=1.0)
