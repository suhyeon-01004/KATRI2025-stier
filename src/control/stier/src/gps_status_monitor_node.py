#!/usr/bin/env python3
import rospy
from ublox_msgs.msg import NavSAT, NavPVT
from std_msgs.msg import String

class GPSStatusMonitor:
    def __init__(self):
        rospy.init_node('gps_status_monitor')

        rospy.Subscriber('/ublox_position_receiver/navsat', NavSAT, self.sat_callback)
        rospy.Subscriber('/ublox_position_receiver/navpvt', NavPVT, self.pvt_callback)

        self.fix_type = 0
        self.status_pub = rospy.Publisher('/gps_status_info', String, queue_size=10)

    def pvt_callback(self, msg):
        self.fix_type = msg.fixType

    def sat_callback(self, msg):
        total_cno = 0
        used_satellites = 0
        diff_corr_count = 0
        satellite_count = len(msg.sv)
        cno_list = []

        for sat in msg.sv:
            total_cno += sat.cno
            cno_list.append(sat.cno)
            if sat.flags & (1 << 3):  # svUsed bit check
                used_satellites += 1
            if sat.flags & (1 << 6):  # diffCorr bit check
                diff_corr_count += 1

        avg_cno = total_cno / satellite_count if satellite_count else 0

        reason = []
        if used_satellites >= 10 and avg_cno >= 35 and diff_corr_count >= 3:
            status = "RTK Fix 가능성 높음"
            reason.append("- 사용 가능 위상 개수: {} 개".format(used_satellites))
            reason.append("- 포착 위상 타겟 C/N0 평균: {:.2f} dBHz".format(avg_cno))
            reason.append("- 보정 정보가 적용된 위상: {} 개".format(diff_corr_count))
        elif used_satellites >= 6 and avg_cno >= 30:
            status = "일반 GPS Fix는 가능, RTK는 불확실"
            if used_satellites < 10:
                reason.append("- 사용 가능 위상 개수가 RTK 가능 하기에 모양")
            if avg_cno < 35:
                reason.append("- 위상 시간의 총 C/N0 평균가 RTK 복귀에 다르기에 모양")
            reason.append("- 평균 C/N0: {:.2f} dBHz, 위상 수: {}, 보정 위상 수: {}".format(avg_cno, used_satellites, diff_corr_count))
        else:
            status = "GPS 상태 불안정"
            if used_satellites < 6:
                reason.append("- 사용 가능 위상 개수가 너무 적음 ({})".format(used_satellites))
            if avg_cno < 30:
                reason.append("- 위상 C/N0 평균가 너무 낮음 ({:.2f} dBHz)".format(avg_cno))
            reason.append("- 보정 적용 위상 수: {}".format(diff_corr_count))

        # 추가 경고 메시지
        if avg_cno < 25:
            reason.append("- 경고: GPS 신호가 매우 약합니다. 실내이거나 차폐된 환경일 수 있습니다.")
            reason.append("- 권장: 개방된 야외 환경으로 이동하거나 안테나 상태를 점검하세요.")

        if used_satellites < 4:
            reason.append("- 경고: 위성 수가 너무 적어 RTK는 물론 GPS 고정도 어려울 수 있습니다.")
            reason.append("- 권장: 안테나 위치나 주변 장애물 여부를 확인해보세요.")

        if self.fix_type == 5:
            status += " [현재 RTK Fixed 상태!]"

        output_msg = f"[GPS Status] {status}\n"
        output_msg += "\n".join(reason)
        output_msg += f"\n- 개수 포착된 위상: {satellite_count} 개\n"
        output_msg += f"- C/N0 값 (가공): {cno_list}"

        print(output_msg)
        self.status_pub.publish(output_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    print("Hi")
    monitor = GPSStatusMonitor()
    monitor.run()