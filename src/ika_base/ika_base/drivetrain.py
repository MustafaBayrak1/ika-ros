"""Motor + enkoder donanımını tek arayüzde toplar.

    dt = build_drivetrain(cfg)      # cfg: config.DEFAULTS biçiminde sözlük
    dt.set_duty(sol, sag)           # -100..100
    dt.set_duty(0, 0, brake=(60, 60))  # duran motorları fren gücüyle tut (brake_on_stop)
    dt.step(dt_s)                   # sadece benzetimde iş yapar
    dt.counts()                     # (sol, sag) kümülatif enkoder sayımı veya None
    dt.close()

GpioDrivetrain gerçek donanımı, SimDrivetrain (backend: mock) basit bir
motor modeliyle enkoder üretir; üst katman ikisini ayırt etmez.
"""

import functools
import math

from ika_base.encoder import EncoderReader, ProcessEncoderReader, multiplier
from ika_hal.gpio import open_chip, resolve_pin
from ika_hal.pwm import open_pwm


class HBridgeMotor:
    """in_in_pwm: IN1/IN2 yön, EN hattına PWM (L298N, TB6612, ...)
    dir_pwm:   tek DIR hattı + PWM (in2 kullanılmaz)
    PWM yoksa (EN jumper'lı) motor yalnızca tam hız / dur çalışır."""

    def __init__(self, in1, in2, pwm, mode='in_in_pwm', invert=False, brake=False):
        if mode not in ('in_in_pwm', 'dir_pwm'):
            raise ValueError(f"motor_driver in_in_pwm veya dir_pwm olmalı, gelen: {mode!r}")
        if in1 is None or (mode == 'in_in_pwm' and in2 is None):
            raise ValueError(f'{mode} için yön pinleri tanımlanmalı')
        self._in1 = in1      # (request, offset)
        self._in2 = in2
        self._pwm = pwm
        self._mode = mode
        self._invert = invert
        self._brake = brake

    @staticmethod
    def _set(pin, value):
        if pin is not None:
            pin[0].set(pin[1], value)

    def drive(self, duty: float, brake: float = 0.0):
        """duty: -100..100. duty 0 iken brake > 0 ve brake_on_stop açıksa
        (in_in_pwm) motor kısa devre frenine alınır: IN1 = IN2 = 1 ve PWM'e
        fren gücü [%] verilir. L298N'de fren için EN'in yüksek olması gerekir;
        eskiden EN %0'da kalıyor, motor fren yerine serbest dönüyordu."""
        if self._invert:
            duty = -duty
        if self._pwm is None:
            duty = math.copysign(100.0, duty) if abs(duty) >= 20.0 else 0.0
        forward, backward = duty > 0.0, duty < 0.0
        hold = self._brake and brake > 0.0 and not (forward or backward)
        if self._mode == 'dir_pwm':
            self._set(self._in1, 1 if forward else 0)
            hold = False
        elif forward or backward:
            self._set(self._in1, 1 if forward else 0)
            self._set(self._in2, 1 if backward else 0)
        else:
            self._set(self._in1, 1 if hold else 0)
            self._set(self._in2, 1 if hold else 0)
        if self._pwm is not None:
            self._pwm.set_duty_percent(min(100.0, brake) if hold else abs(duty))

    def close(self):
        self.drive(0.0)
        if self._pwm is not None:
            self._pwm.release()


class GpioDrivetrain:

    def __init__(self, cfg: dict, backend: str = 'gpiod'):
        self._chips = {}
        self._requests = []
        self.encoders = None
        self.motors = []

        pins = {key: resolve_pin(cfg[key], backend) for key in (
            'left.in1', 'left.in2', 'right.in1', 'right.in2',
            'left.enc_a', 'left.enc_b', 'right.enc_a', 'right.enc_b')}
        used = [p for p in pins.values() if p]
        if len(set(used)) != len(used):
            raise ValueError(f'aynı GPIO hattı iki kez kullanılmış: {pins}')

        # Tüm çıkışları chip başına tek istekte topla.
        outputs = {}
        for side in ('left', 'right'):
            for key in ('in1', 'in2'):
                pin = pins[f'{side}.{key}']
                if pin:
                    outputs.setdefault(pin[0], []).append(pin[1])
        out_req = {}
        for chip_name, offsets in outputs.items():
            req = self._chip(chip_name, backend).request_outputs(offsets, consumer='ika_base')
            self._requests.append(req)
            out_req[chip_name] = req

        def out_pin(key):
            pin = pins[key]
            return (out_req[pin[0]], pin[1]) if pin else None

        self.notes = []
        for side in ('left', 'right'):
            pwm = open_pwm(cfg[f'{side}.pwm'], cfg['pwm_period_ns'], backend)
            if pwm is not None and pwm.note:
                self.notes.append(f'{side}: {pwm.note}')
            self.motors.append(HBridgeMotor(
                out_pin(f'{side}.in1'), out_pin(f'{side}.in2'), pwm,
                mode=cfg['motor_driver'], invert=cfg[f'{side}.invert'],
                brake=cfg['brake_on_stop']))

        if cfg['encoder.mode'] != 'none':
            encoders = []
            for side in ('left', 'right'):
                a, b = pins[f'{side}.enc_a'], pins[f'{side}.enc_b']
                if a is None or b is None:
                    raise ValueError(f'{side} enkoder pinleri (enc_a / enc_b) tanımlanmalı '
                                     "veya encoder.mode: none verilmeli")
                encoders.append((a, b, cfg[f'{side}.enc_invert']))
            if backend == 'gpiod':
                # Ayrı süreç: bu süreçteki ROS döngüsüyle GIL paylaşılmaz.
                self.encoders = ProcessEncoderReader(
                    functools.partial(open_chip, backend=backend), encoders,
                    mode=cfg['encoder.mode'], bias=cfg['encoder.bias'],
                    rt_priority=int(cfg['encoder.rt_priority']))
                if self.encoders.note:
                    self.notes.append(self.encoders.note)
            else:
                self.encoders = EncoderReader(
                    lambda name: self._chip(name, backend), encoders,
                    mode=cfg['encoder.mode'], bias=cfg['encoder.bias'])

    def _chip(self, name, backend):
        if name not in self._chips:
            self._chips[name] = open_chip(name, backend)
        return self._chips[name]

    def set_duty(self, left: float, right: float, brake=(0.0, 0.0)):
        self.motors[0].drive(left, brake[0])
        self.motors[1].drive(right, brake[1])

    def step(self, dt: float):
        pass

    def counts(self):
        return tuple(self.encoders.counts()) if self.encoders else None

    def encoder_errors(self):
        return tuple(self.encoders.errors()) if self.encoders else (0, 0)

    def encoder_alive(self) -> bool:
        alive = getattr(self.encoders, 'alive', None)
        return alive() if alive else True

    def close(self):
        for motor in self.motors:
            motor.close()
        if self.encoders:
            self.encoders.close()
        for req in self._requests:
            req.release()


class SimDrivetrain:
    """Donanımsız benzetim: teker hızı duty'ye birinci dereceden yanıt verir
    (tau), altına küçük bir ölü bölge ve sol/sağ verim farkı eklenir; PID
    gerçekten bir şeyi düzeltiyor olsun diye."""

    def __init__(self, cfg: dict, tau: float = 0.12, dead_duty: float = 8.0,
                 right_gain: float = 0.93):
        self._max_speed = cfg['max_wheel_speed']
        self._cpr = cfg['encoder.ppr'] * multiplier(cfg['encoder.mode']) or 1000.0
        self._radius = cfg['wheel_radius']
        self._tau = tau
        self._dead = dead_duty
        self._gains = (1.0, right_gain)
        self._duty = [0.0, 0.0]
        self._speed = [0.0, 0.0]
        self._brake = [0.0, 0.0]
        self._pos = [0.0, 0.0]
        self.counts_per_rev = self._cpr

    def set_duty(self, left: float, right: float, brake=(0.0, 0.0)):
        self._duty = [left, right]
        self._brake = list(brake)

    def step(self, dt: float):
        for i in range(2):
            duty = self._duty[i]
            effective = 0.0 if abs(duty) < self._dead else duty
            target = effective / 100.0 * self._max_speed * self._gains[i]
            # Fren (duty 0 + brake): teker serbest dönüşten ~4 kat hızlı durur.
            tau = self._tau / 4.0 if duty == 0.0 and self._brake[i] > 0.0 else self._tau
            self._speed[i] += (target - self._speed[i]) * min(1.0, dt / tau)
            self._pos[i] += self._speed[i] * dt

    def counts(self):
        per_m = self._cpr / (2.0 * math.pi * self._radius)
        return int(self._pos[0] * per_m), int(self._pos[1] * per_m)

    def encoder_errors(self):
        return (0, 0)

    def encoder_alive(self) -> bool:
        return True

    def close(self):
        pass


def build_drivetrain(cfg: dict):
    if cfg['backend'] == 'mock':
        return SimDrivetrain(cfg)
    return GpioDrivetrain(cfg, cfg['backend'])
