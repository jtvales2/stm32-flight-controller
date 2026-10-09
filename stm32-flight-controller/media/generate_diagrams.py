"""Regenerate the code-native SVGs with Python 3; no external packages required."""
from pathlib import Path
from html import escape

OUT = Path(__file__).resolve().parent
NAVY = '#152c46'
BLUE = '#2474b7'
TEAL = '#237b77'
MUTED = '#54667a'
WARM = '#faf8f3'
BORDER = '#d9e1e8'

class Diagram:
    def __init__(self, name, height, title, subtitle):
        self.name, self.height, self.items = name, height, []
        self.items.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="1600" height="{height}" viewBox="0 0 1600 {height}" role="img" aria-labelledby="title desc">')
        self.items.append(f'<title id="title">{escape(title)}</title><desc id="desc">{escape(subtitle)} Code-derived conceptual flow; not measured hardware or flight evidence.</desc>')
        self.items.append('<defs><marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="9" markerHeight="9" orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="#2474b7"/></marker><marker id="teal" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="9" markerHeight="9" orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="#237b77"/></marker></defs>')
        self.rect(0, 0, 1600, height, WARM, stroke='none', radius=0)
        self.text(60, 62, 'STM32 FLIGHT CONTROLLER', 22, BLUE, weight=700, spacing=3)
        self.text(60, 128, title, 49, NAVY, weight=700)
        self.text(60, 176, subtitle, 24, MUTED)
    def rect(self, x, y, w, h, fill='white', stroke=BORDER, radius=20):
        self.items.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{radius}" fill="{fill}" stroke="{stroke}" stroke-width="2"/>')
    def text(self, x, y, s, size=24, color=NAVY, weight=400, spacing=0, mono=False):
        family = 'Consolas, monospace' if mono else 'Segoe UI, Arial, sans-serif'
        self.items.append(f'<text x="{x}" y="{y}" font-family="{family}" font-size="{size}" fill="{color}" font-weight="{weight}" letter-spacing="{spacing}">{escape(s)}</text>')
    def lines(self, x, y, lines, size=24, color=MUTED, gap=34):
        for i, s in enumerate(lines): self.text(x, y+i*gap, s, size, color)
    def card(self, x, y, w, h, title, body, source=None, fill='white', accent=BLUE):
        self.rect(x, y, w, h, fill)
        self.rect(x+22, y+25, 5, 32, accent, stroke='none', radius=2)
        compact = h <= 140
        self.text(x+43, y+(43 if compact else 51), title, 27, NAVY, 700)
        self.lines(x+28, y+(77 if compact else 94), body, 23 if compact else 24, MUTED, 32)
        if source: self.text(x+28, y+h-(15 if compact else 24), source, 20, MUTED, mono=True)
    def arrow(self, pts, color=BLUE, dashed=False):
        path = 'M '+ ' L '.join(f'{x} {y}' for x,y in pts)
        dash=' stroke-dasharray="9 7"' if dashed else ''
        mark='teal' if color==TEAL else 'arrow'
        self.items.append(f'<path d="{path}" fill="none" stroke="{color}" stroke-width="3.5" stroke-linejoin="round"{dash} marker-end="url(#{mark})"/>')
    def footer(self, s):
        self.text(60, self.height-38, s, 21, MUTED)
    def save(self):
        self.items.append('</svg>')
        (OUT/f'{self.name}.svg').write_text('\n'.join(self.items)+'\n', encoding='utf-8')

d = Diagram('system-architecture', 1020, 'From sensor events to motor commands', 'STM32F407ZGTx · BMI088 · SBUS · MS5611 · four PWM outputs')
d.text(60, 235, 'INPUTS', 21, MUTED, 700, 2)
d.text(460, 235, 'MAIN LOOP + INTERRUPT CALLBACKS', 21, MUTED, 700, 2)
d.text(1200, 235, 'ACTUATION', 21, MUTED, 700, 2)
d.card(60, 266, 340, 136, 'BMI088', ['Accel + gyro DRDY / SPI1'], 'pipeline.c')
d.card(60, 426, 340, 136, 'SBUS receiver', ['USART6 circular DMA'], 'sbus.c → fc_rc.c')
d.card(60, 586, 340, 136, 'MS5611 barometer', ['Scheduled SPI conversions'], 'ms5611.c → fc_baro.c')
d.rect(460, 266, 680, 456, '#eef4f8')
d.card(488, 290, 624, 118, 'Asynchronous acquisition', ['DMA arbitration → timestamped rings'], 'pipeline.c · ringbuf_spsc.c')
d.card(488, 439, 624, 118, 'Timestamp pairing + attitude', ['Causal accel pairing → 6-DoF Mahony'], 'sync_pair.c · fusion_mahony.c')
d.card(488, 588, 624, 124, 'Controller integration', ['RC / arming / health gates → control'], 'fc_core.c → fc_control.c')
d.card(1200, 440, 340, 258, 'Motor output', ['Angle / rate / yaw', 'Throttle / altitude hold', 'Mixer + output limits', 'TIM3 PWM × 4'], 'fc_mixer_out.c · motors.c')
d.arrow([(400,334),(488,334)])
d.arrow([(800,408),(800,439)])
d.arrow([(800,557),(800,588)])
d.arrow([(400,494),(430,494),(430,624),(488,624)])
d.arrow([(400,654),(488,654)])
d.arrow([(1112,643),(1166,643),(1166,569),(1200,569)])
d.rect(60, 788, 1480, 138, NAVY, stroke='none')
d.text(88, 833, 'SUPERVISION', 22, '#91caef', 700, 2)
d.text(330, 833, 'TIM2 1 kHz watchdog · IMU health · RC validity · tilt latch', 26, '#ffffff', 700)
d.text(88, 884, 'Latched faults disarm and reset controller state; stale barometer data exits altitude hold.', 24, '#d9e8f4')
d.arrow([(1370,788),(1370,710)], dashed=True)
d.footer('Code map: main.c / fc_core.c connect the modules. Diagram describes implementation, not validation results.')
d.save()

d = Diagram('imu-pipeline', 1110, 'A timestamped, gyro-driven IMU pipeline', 'IRQ timestamps travel with asynchronous samples; fusion timing follows gyro timestamps.')
cards = [
 (60,250,'01  DRDY capture',['Timestamp accel / gyro edges','Apply deglitch thresholds','Publish latest pending event'],'main.c · pipeline.c'),
 (580,250,'02  SPI DMA arbitration',['Choose one sensor transfer','Prefer gyro; serve pending accel','after 3 completed gyro reads'],'pipeline.c'),
 (1100,250,'03  Frontend + data rings',['Scale / bias / filter raw vectors','Push timestamped accel + gyro','Drop oldest on overflow'],'imu_bmi088_frontend.c'),
 (1100,620,'04  Causal time pairing',['Pop oldest gyro; derive dt','Use latest accel at or before gyro','Reject reversed gyro timestamps'],'sync_pair.c'),
 (580,620,'05  Mahony 6-DoF',['Update quaternion from gyro','Use accel only within age limit','Expose Euler attitude + gyro'],'fusion_mahony.c'),
 (60,620,'06  Control scheduling',['Select budget from gyro queue','Catch up when sample lag grows','Step control using derived dt'],'main.c → fc_core.c'),
]
for x,y,title,body,source in cards: d.card(x,y,440,220,title,body,source)
d.arrow([(500,360),(580,360)])
d.arrow([(1020,360),(1100,360)])
d.arrow([(1320,470),(1320,620)])
d.arrow([(1100,730),(1020,730)])
d.arrow([(580,730),(500,730)])
d.text(60, 516, 'Pending IRQs are coalesced.', 25, NAVY, 700)
d.lines(60, 552, ['Latest timestamps are retained; merge-drop counters record skipped events.'], 24)
d.text(1100, 530, 'ringbuf_spsc.c', 21, MUTED, mono=True)
d.rect(60, 913, 1480, 105, NAVY, stroke='none')
d.text(88, 954, 'OBSERVABILITY', 22, '#91caef', 700, 2)
d.text(358, 954, 'IRQ intervals · queue depth · merge / sweep drops · pairing age · DMA timeouts', 23, '#ffffff')
d.text(88, 991, 'Configured rates and limits describe code policy; measured timing evidence remains separate.', 23, '#d9e8f4')
d.footer('Sources: pipeline.c / ringbuf_spsc.c / sync_pair.c / fusion_mahony.c / main.c. No synthetic measurements.')
d.save()

d = Diagram('fault-handling', 1240, 'Recovery has distinct boundaries', 'A peripheral restart, a latched flight stop and an altitude-hold exit have different consequences.')
for x,title in [(60,'ACQUISITION RECOVERY'),(580,'LATCHED FLIGHT STOP'),(1100,'ALTITUDE-HOLD EXIT')]:
    d.text(x, 245, title, 21, MUTED, 700, 1.5)
d.card(60, 284, 440, 185, 'SPI / DMA interruption', ['SPI error callback, or', 'in-flight DMA longer than 10 ms'], 'pipeline.c')
d.card(60, 520, 440, 220, 'Release and restart', ['Release chip select; stop DMA','Timeout: reset SPI ready state','Clear in-flight state; request kick'], 'imu_pipeline_dma_poll()',accent=TEAL)
d.arrow([(280,469),(280,520)], TEAL)
d.arrow([(500,630),(540,630),(540,827),(280,827),(280,752)], TEAL, dashed=True)
d.lines(60, 892, ['Polling resumes pending transfers.', 'More than 3 DMA timeouts in the', '1-second window while armed', 'also triggers a latched IMU stop.'], 24)
d.card(580, 284, 440, 185, 'Fault detectors', ['RC failsafe / timeout; IMU health','Control stall / hard lag; tilt'], 'fc_rc.c · fc_time.c · fc_fs.c')
d.card(580, 520, 440, 180, 'Emergency stop', ['Latch reason; set armed = 0','Reset control; write stop pulses'], 'fc_emergency_stop()')
d.arrow([(800,469),(800,520)])
d.card(580, 750, 440, 210, 'Deliberate latch clear', ['ARM off held > 300 ms','Low throttle + level + stable RC','Clearing keeps motors disarmed'], 'fc_arm.c → fc_fs.c')
d.arrow([(800,700),(800,750)])
d.card(580, 987, 440, 185, 'Fresh ARM 0 → 1', ['AHRS reset complete; no latch','Stable RC + low throttle + level'], 'fc_arm_update()')
d.arrow([(800,960),(800,987)])
d.card(1100, 284, 440, 185, 'Stale barometer data', ['TIM2 watchdog flags stale data','Main loop services pending flag'], 'fc_core.c')
d.card(1100, 520, 440, 220, 'Exit and reset ALT_HOLD', ['Invalidate barometer snapshot','Reset altitude controller state','Record exit if previously active'], 'fc_core_service_async()',accent=TEAL)
d.arrow([(1320,469),(1320,520)], TEAL)
d.card(1100, 790, 440, 210, 'Guarded re-entry', ['Valid barometer data + entry gates','ALT switch OFF → ON required','Attitude / rate control retained'], 'fc_alt_hold.c',accent=TEAL)
d.arrow([(1320,740),(1320,790)], TEAL)
d.lines(1100, 1034, ['Sensor data returning does not', 'automatically re-arm the aircraft.'], 24, NAVY)
d.footer('Code-derived flow. Flight-stop detectors are conditional on their code gates; hardware fault injection is pending.')
d.save()
