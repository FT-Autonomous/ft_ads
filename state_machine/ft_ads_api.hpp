#ifndef state_machine_ft_ads_api_h
#define state_machine_ft_ads_api_h

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


typedef enum ft_ads_api_as_state_e {
	AS_OFF = 1,
	AS_READY = 2,
	AS_DRIVING = 3,
	AS_EMERGENCY_BRAKE = 4,
	AS_FINISHED = 5,
} ft_ads_api_as_state_e;


typedef enum ft_ads_api_ami_state_e {
	AMI_NOT_SELECTED = 0,
	AMI_ACCELERATION = 1,
	AMI_SKIDPAD = 2,
	AMI_AUTOCROSS = 3,
	AMI_TRACK_DRIVE = 4,
	AMI_STATIC_INSPECTION_A = 5,
	AMI_STATIC_INSPECTION_B = 6,
	AMI_AUTONOMOUS_DEMO = 7,
} ft_ads_api_ami_state_e;


typedef enum ft_ads_api_handshake_receive_bit_e {
	HANDSHAKE_RECEIVE_BIT_OFF = 0,
	HANDSHAKE_RECEIVE_BIT_ON = 1,
} ft_ads_api_handshake_receive_bit_e;


typedef enum ft_ads_api_res_go_signal_bit_e {
	RES_GO_SIGNAL_NO_GO = 0,
	RES_GO_SIGNAL_GO = 1,
} ft_ads_api_res_go_signal_bit_e;


#ifdef __cplusplus
typedef volatile struct alignas(4) ft_ads_api_vcu2ai_struct {
	volatile ft_ads_api_handshake_receive_bit_e	VCU2AI_HANDSHAKE_RECEIVE_BIT;
	volatile ft_ads_api_res_go_signal_bit_e		VCU2AI_RES_GO_SIGNAL;
	volatile ft_ads_api_as_state_e				VCU2AI_AS_STATE;
	volatile ft_ads_api_ami_state_e				VCU2AI_AMI_STATE;
	volatile float								VCU2AI_STEER_ANGLE_deg;
	volatile float								VCU2AI_BRAKE_PRESS_F_pct;
	volatile float								VCU2AI_BRAKE_PRESS_R_pct;
	volatile float								VCU2AI_RL_WHEEL_SPEED_rpm;
	volatile float								VCU2AI_RR_WHEEL_SPEED_rpm;
	volatile uint16_t							VCU2AI_RL_PULSE_COUNT;
	volatile uint16_t							VCU2AI_RR_PULSE_COUNT;
} ft_ads_api_vcu2ai;
#else
typedef volatile struct ft_ads_api_vcu2ai_struct {
	volatile _Alignas(4) ft_ads_api_handshake_receive_bit_e	VCU2AI_HANDSHAKE_RECEIVE_BIT;
	volatile _Alignas(4) ft_ads_api_res_go_signal_bit_e		VCU2AI_RES_GO_SIGNAL;
	volatile _Alignas(4) ft_ads_api_as_state_e				VCU2AI_AS_STATE;
	volatile _Alignas(4) ft_ads_api_ami_state_e				VCU2AI_AMI_STATE;
	volatile _Alignas(4) float								VCU2AI_STEER_ANGLE_deg;
	volatile _Alignas(4) float								VCU2AI_BRAKE_PRESS_F_pct;
	volatile _Alignas(4) float								VCU2AI_BRAKE_PRESS_R_pct;
	volatile _Alignas(4) float								VCU2AI_FL_WHEEL_SPEED_rpm;
	volatile _Alignas(4) float								VCU2AI_FR_WHEEL_SPEED_rpm;
	volatile _Alignas(4) float								VCU2AI_RL_WHEEL_SPEED_rpm;
	volatile _Alignas(4) float								VCU2AI_RR_WHEEL_SPEED_rpm;
	volatile _Alignas(4) uint16_t							VCU2AI_FL_PULSE_COUNT;
	volatile _Alignas(4) uint16_t							VCU2AI_FR_PULSE_COUNT;
	volatile _Alignas(4) uint16_t							VCU2AI_RL_PULSE_COUNT;
	volatile _Alignas(4) uint16_t							VCU2AI_RR_PULSE_COUNT;
} ft_ads_api_vcu2ai;
#endif

typedef enum ft_ads_api_mission_status_e {
	MISSION_NOT_SELECTED = 0,
	MISSION_SELECTED = 1,
	MISSION_RUNNING = 2,
	MISSION_FINISHED = 3,
} ft_ads_api_mission_status_e;


typedef enum ft_ads_api_direction_request_e {
	DIRECTION_NEUTRAL = 0,
	DIRECTION_FORWARD = 1,
} ft_ads_api_direction_request_e;


typedef enum ft_ads_api_estop_request_e {
	ESTOP_NO = 0,
	ESTOP_YES = 1,
} ft_ads_api_estop_request_e;


typedef enum ft_ads_api_handshake_send_bit_e {
	HANDSHAKE_SEND_BIT_OFF = 0,
	HANDSHAKE_SEND_BIT_ON = 1,
} ft_ads_api_handshake_send_bit_e;

#ifdef __cplusplus
typedef volatile struct alignas(4) ft_ads_api_ai2vcu_struct {
	volatile ft_ads_api_mission_status_e		AI2VCU_MISSION_STATUS;
	volatile ft_ads_api_direction_request_e	AI2VCU_DIRECTION_REQUEST;
	volatile ft_ads_api_estop_request_e		AI2VCU_ESTOP_REQUEST;
	volatile ft_ads_api_handshake_send_bit_e	AI2VCU_HANDSHAKE_SEND_BIT;
	volatile float							AI2VCU_STEER_ANGLE_REQUEST_deg;
	volatile float							AI2VCU_AXLE_SPEED_REQUEST_rpm;
	volatile float							AI2VCU_AXLE_GEAR_SHIFT_REQUEST_int;
	volatile float							AI2VCU_BRAKE_PRESS_REQUEST_pct;
} ft_ads_api_ai2vcu;
#else
typedef volatile struct ft_ads_api_ai2vcu_struct {
	volatile _Alignas(4) ft_ads_api_mission_status_e		AI2VCU_MISSION_STATUS;
	volatile _Alignas(4) ft_ads_api_direction_request_e	AI2VCU_DIRECTION_REQUEST;
	volatile _Alignas(4) ft_ads_api_estop_request_e		AI2VCU_ESTOP_REQUEST;
	volatile _Alignas(4) ft_ads_api_handshake_send_bit_e	AI2VCU_HANDSHAKE_SEND_BIT;
	volatile _Alignas(4) float							AI2VCU_STEER_ANGLE_REQUEST_deg;
	volatile _Alignas(4) float							AI2VCU_AXLE_SPEED_REQUEST_rpm;
	volatile _Alignas(4) float							AI2VCU_AXLE_GEAR_SHIFT_REQUEST_int;
	volatile _Alignas(4) float							AI2VCU_BRAKE_PRESS_REQUEST_pct;
} ft_ads_api_ai2vcu;
#endif

// typedef struct can_stats_struct {
// 	volatile uint32_t VCU2AI_Status_count;
// 	volatile uint32_t VCU2AI_Drive_F_count;
// 	volatile uint32_t VCU2AI_Drive_R_count;
// 	volatile uint32_t VCU2AI_Steer_count;
// 	volatile uint32_t VCU2AI_Brake_count;
// 	volatile uint32_t VCU2AI_Wheel_speeds_count;
// 	volatile uint32_t VCU2AI_Wheel_counts_count;
// 	volatile uint32_t PCAN_GPS_BMC_Acceleration_count;
// 	volatile uint32_t PCAN_GPS_BMC_MagneticField_count;
// 	volatile uint32_t PCAN_GPS_L3GD20_Rotation_A_count;
// 	volatile uint32_t PCAN_GPS_L3GD20_Rotation_B_count;
// 	volatile uint32_t PCAN_GPS_GPS_Status_count;
// 	volatile uint32_t PCAN_GPS_GPS_CourseSpeed_count;
// 	volatile uint32_t PCAN_GPS_GPS_Longitude_count;
// 	volatile uint32_t PCAN_GPS_GPS_Latitude_count;
// 	volatile uint32_t PCAN_GPS_GPS_Altitude_count;
// 	volatile uint32_t PCAN_GPS_GPS_Delusions_A_count;
// 	volatile uint32_t PCAN_GPS_GPS_Delusions_B_count;
// 	volatile uint32_t PCAN_GPS_GPS_DateTime_count;
// 	volatile uint32_t unhandled_frame_count;
// } can_stats_t;

/*******************************************************************************************
    ****************** TODO: Figure out what protocol we are using ******************
*******************************************************************************************/
typedef struct ethernet_stats_struct {
    volatile uint32_t VCU2AI_Status_count;
    volatile uint32_t VCU2AI_Drive_F_count;
    volatile uint32_t VCU2AI_Drive_R_count;
    volatile uint32_t VCU2AI_Steer_count;
    volatile uint32_t VCU2AI_Brake_count;
    volatile uint32_t VCU2AI_Wheel_speeds_count;
    volatile uint32_t VCU2AI_Wheel_counts_count;
    volatile uint32_t unhandled_frame_count;
} ethernet_stats_t;

int ft_ads_api_init(char *CAN_interface, int debug, int simulate);

void ft_ads_api_vcu2ai_get_data(ft_ads_api_vcu2ai *data);
void ft_ads_api_ai2vcu_set_data(ft_ads_api_ai2vcu *data);

void ft_ads_api_get_can_stats(ethernet_stats_t *data);
void ft_ads_api_clear_can_stats();

#ifdef __cplusplus
}
#endif

#endif /* state_machine_ft_ads_api_h */
