#!/usr/bin/env bash
# Robotun teker odometrisinden (/wheel/odom) canlı mesafe ölçer.
# Robotta ya da laptopta (ika servisi çalışırken) aynı komutla çalışır:
#   bash ~/mesafe_olc.sh
# Laptopta robota SSH ile bağlanıp robottaki kopyayı başlatır; robotun adresi
# değiştiyse: IKA_ROBOT=<ip> bash ~/mesafe_olc.sh
# Başladığı andaki konumu sıfır kabul eder. Ctrl+C ile durur ve özeti yazar.
if [ ! -f /etc/default/ika ]; then
  exec ssh -t "gemstone@${IKA_ROBOT:-10.43.0.67}" bash mesafe_olc.sh
fi
source /opt/ros/jazzy/setup.bash
[ -f /etc/default/ika ] && set -a && source /etc/default/ika && set +a
# Servis root olarak çalışıyor; paylaşımlı bellek yolu kullanıcılar arasında
# veri taşımadığı için UDP'ye zorluyoruz (yoksa konular görünür ama veri gelmez).
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

exec python3 - <<'EOF'
import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

SICRAMA = 0.2  # 50 Hz'te tek adımda bundan uzun yol = odometri sıfırlandı


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class MesafeOlcer(Node):
    def __init__(self):
        super().__init__('mesafe_olc')
        self.start = self.last = self.last_yaw = None
        self.uzaklik = self.yol = self.donus = 0.0
        self.teker = None
        self.mesaj_var = False
        self.create_subscription(Odometry, '/wheel/odom', self.on_odom, 20)
        self.create_subscription(Float32MultiArray, '/ika_base_node/wheels', self.on_wheels, 20)
        self.create_timer(0.5, self.yaz)
        self.create_timer(3.0, self.kontrol)

    def on_odom(self, m):
        self.mesaj_var = True
        p, yaw = m.pose.pose.position, yaw_of(m.pose.pose.orientation)
        if self.start is None or math.hypot(p.x - self.last[0], p.y - self.last[1]) > SICRAMA:
            if self.start is not None:
                print('\nOdometri sıfırlandı; ölçüm yeni konumdan devam ediyor.')
                self.start = (p.x - self.uzaklik_vektoru[0], p.y - self.uzaklik_vektoru[1])
            else:
                self.start = (p.x, p.y)
            self.last, self.last_yaw = (p.x, p.y), yaw
            return
        self.yol += math.hypot(p.x - self.last[0], p.y - self.last[1])
        self.donus += math.atan2(math.sin(yaw - self.last_yaw), math.cos(yaw - self.last_yaw))
        self.last, self.last_yaw = (p.x, p.y), yaw
        self.uzaklik = math.hypot(p.x - self.start[0], p.y - self.start[1])

    @property
    def uzaklik_vektoru(self):
        return (self.last[0] - self.start[0], self.last[1] - self.start[1])

    def on_wheels(self, m):
        if len(m.data) >= 5:
            self.teker = (m.data[1], m.data[4])

    def kontrol(self):
        if not self.mesaj_var:
            print('\n/wheel/odom verisi gelmiyor. ika servisi çalışıyor mu? '
                  '(systemctl is-active ika)')

    def satir(self):
        teker = ('—' if self.teker is None
                 else f'{self.teker[0]:+.2f} / {self.teker[1]:+.2f} m/s')
        return (f'Başlangıca uzaklık: {self.uzaklik:5.2f} m | Toplam yol: {self.yol:5.2f} m'
                f' | Dönüş: {math.degrees(self.donus):+7.1f}° | Teker sol/sağ: {teker}')

    def yaz(self):
        if self.start is not None:
            sys.stdout.write('\r' + self.satir() + '   ')
            sys.stdout.flush()


rclpy.init()
node = MesafeOlcer()
print('Ölçüm başladı. Robotu sürün; durdurmak için Ctrl+C.')
try:
    rclpy.spin(node)
except (KeyboardInterrupt, ExternalShutdownException):
    pass
print('\n\nSonuç: ' + node.satir())
print('Dönüş: + sola (saat yönünün tersi), − sağa. 360° = bir tam tur.')
node.destroy_node()
rclpy.try_shutdown()
EOF
