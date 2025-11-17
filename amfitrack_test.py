import amfiprot
import amfiprot_amfitrack as amfitrack
from loop_rate_limiters import RateLimiter

VENDOR_ID = 0xC17
PRODUCT_ID_SENSOR = 0xD12
PRODUCT_ID_SOURCE = 0xD01

if __name__ == "__main__":
    conn = None
    try:
        conn = amfiprot.USBConnection(VENDOR_ID, PRODUCT_ID_SENSOR)
    except:
        try:
            conn = amfiprot.USBConnection(VENDOR_ID, PRODUCT_ID_SOURCE)
        except:
            print("No Amfitrack device found")
            exit()
            
    nodes = conn.find_nodes()
    Devs = []

    print(f"Found {len(nodes)} node(s).")
    for node in nodes:
        print(f"[{node.tx_id}] {node.name}")
        Devs.append(amfitrack.Device(node))
    
    conn.start()
    # '''
    
    while True:
        for idx, Dev in enumerate(Devs):
            if Dev.packet_available():
                packet = Dev.get_packet()
                if type(packet.payload) == amfitrack.payload.EmfImuFrameIdPayload:
                    # Sensor measurement package 
                    payload: amfitrack.payload.EmfImuFrameIdPayload = packet.payload
                    print(payload.emf, Dev.name()) # emf contains position and orientation
                elif type(packet.payload) == amfitrack.payload.SourceMeasurementPayload:
                    # Source measurement package
                    payload: amfitrack.payload.SourceMeasurementPayload = packet.payload
                    print("Source current:" + str(payload.current))
                elif type(packet.payload) == amfitrack.payload.SourceCalibrationPayload:
                    # Source calibration package (sent at 5Hz)
                    payload: amfitrack.payload.SourceCalibrationPayload = packet.payload
                    print("Source frequency:" + str(payload.frequency))
                else:
                    print(packet)
    # '''
