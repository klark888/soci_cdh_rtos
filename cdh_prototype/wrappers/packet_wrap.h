//methods for constructing and sending packets
bool packet_detect_uplink();
void packet_process_uplink();
bool packet_send_downlink();
void packet_queue_header();
void packet_queue_beacon();
void packet_queue_eps();
void packet_queue_com();
void packet_queue_gnc();
void packet_queue_sen();
void packet_queue_image();
void packet_queue_logger();
void packet_log( char* statement );
