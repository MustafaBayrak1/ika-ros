"""ika_base_node parametreleri: varsayılanlar ve YAML okuma yardımcıları.

Aynı tanımlar hem ROS node'u hem de ROS'suz çalışan hw_check aracı
tarafından kullanılır; böylece ikisi aynı hardware.yaml'ı okur.

Pin: "GPIO22" (Gemstone hat adı) veya "gpiochipN:line". PWM: "ecap0", "epwm1:1".
Boş string = kullanılmıyor.
DİKKAT (ROS 2): float parametrelere YAML'da ondalıklı değer yazın (1 değil 1.0).
"""

DEFAULTS = {
    'backend': 'gpiod',               # gpiod | mock (donanımsız benzetim)
    'motor_driver': 'in_in_pwm',      # in_in_pwm (L298N/TB6612) | dir_pwm
    'brake_on_stop': False,
    'brake_duty': 60.0,               # brake_on_stop: komut sıfırken fren gücü [%]
    'pwm_period_ns': 1_000_000,

    'left.in1': '', 'left.in2': '', 'left.pwm': '', 'left.invert': False,
    'right.in1': '', 'right.in2': '', 'right.pwm': '', 'right.invert': False,

    'encoder.mode': 'x4',             # x4 | x2 | none
    'encoder.bias': 'as-is',          # as-is | pull-up | pull-down | disabled
    'encoder.ppr': 0.0,               # A kanalı darbe / TEKER (çıkış mili) turu
    'encoder.rt_priority': 40,        # okuyucu sürecin SCHED_FIFO önceliği (0: kapalı);
                                      # kontrol döngüsünün IKA_RT_PRIORITY'sinden (30) yüksek olmalı
    'left.enc_a': '', 'left.enc_b': '', 'left.enc_invert': False,
    'right.enc_a': '', 'right.enc_b': '', 'right.enc_invert': False,

    'wheel_radius': 0.0425,           # launch robot.yaml'dan geçirir
    'wheel_separation': 0.17,
    'max_wheel_speed': 0.5,           # %100 duty'deki teker hızı [m/s]
    'max_accel': 0.6,                 # [m/s^2]
    'control_rate': 50.0,
    'state_rate': 10.0,          # /joint_states ve ~/wheels (yalnızca görselleştirme/ayar)
    'cmd_timeout': 0.5,
    'speed_filter_alpha': 0.5,

    'pid.kp': 60.0,                   # [duty % / (m/s)]
    'pid.ki': 300.0,
    'pid.kff': 1.0,
    'pid.min_duty': 0.0,
    'pid.max_duty': 100.0,
    'pid.i_limit': 30.0,              # integralin duty katkısı üst sınırı [%]

    # Teker koruması: |duty| >= stall.duty iken ölçülen hız stall.time boyunca
    # stall.speed altında kalırsa (enkoder susmuş/olay kaçırıyor ya da teker
    # sıkışmış) iki motor da kesilir; komut sıfıra inince yeniden sürülebilir.
    'stall.duty': 70.0,               # [%]; 0: kapalı
    'stall.speed': 0.03,              # [m/s]
    'stall.time': 0.5,                # [s]

    # Düz sürüşte yön tutma: komutta dönüş yokken jiroskopla yönü korur
    # (control.HeadingHold). gyro_sign: jiroskop z'sinin robotun yukarı ekseniyle
    # işareti; kart ters takılıysa (robot.yaml imu.roll = pi) -1.
    'heading_hold.enabled': False,
    'heading_hold.imu_topic': '/imu/data_raw',
    'heading_hold.gyro_sign': 1.0,
    'heading_hold.kp': 1.5,           # [rad/s / rad]
    'heading_hold.ki': 0.5,           # [rad/s / (rad*s)]
    'heading_hold.kd': 0.3,           # [rad/s / (rad/s)]
    'heading_hold.max_w': 0.4,        # düzeltmenin üst sınırı [rad/s]

    'odom_frame': 'odom',
    'base_frame': 'base_link',
    'publish_tf': False,
}


def flatten(tree: dict, prefix: str = '') -> dict:
    flat = {}
    for key, value in tree.items():
        name = f'{prefix}{key}'
        if isinstance(value, dict):
            flat.update(flatten(value, name + '.'))
        else:
            flat[name] = value
    return flat


def load_yaml_params(path: str, node_name: str = 'ika_base_node') -> dict:
    """ROS parametre YAML'ından bu node'un ayarlarını düz sözlük olarak döner."""
    import yaml
    with open(path) as f:
        doc = yaml.safe_load(f) or {}
    section = doc.get(node_name, doc.get(f'/{node_name}', {}))
    cfg = dict(DEFAULTS)
    cfg.update(flatten(section.get('ros__parameters', {})))
    return cfg
