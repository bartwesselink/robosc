/* Headers for the CIF to C translation of controller.cif
 * Generated file, DO NOT EDIT
 */

#ifndef CIF_C_CONTROLLER_ENGINE_H
#define CIF_C_CONTROLLER_ENGINE_H

#include "controller_library.h"

/* Types of the specification.
 * Note that integer ranges are ignored in C.
 */
enum Enumcontroller_ {
    _controller_data_p1KKXDLC4H55M,
    _controller_data_p1UWHO5NYF84Z,
    _controller_in_service,
    _controller_line_found,
    _controller_no_line,
    _controller_none,
    _controller_safe,
    _controller_safe_distance,
    _controller_stopped,
    _controller_unsafe,
    _controller_unsafe_distance,
};
typedef enum Enumcontroller_ controllerEnum;

extern const char *enum_names[];
extern int EnumTypePrint(controllerEnum value, char *dest, int start, int end);


/* Event declarations. */
enum controllerEventEnum_ {
    EVT_INITIAL_,                           /**< Initial step. */
    EVT_DELAY_,                             /**< Delay step. */
    EVT_TAU_,                               /**< Tau step. */
    message_correction_u_response_,         /**< Event message_correction.u_response. */
    data_correction_c_none_,                /**< Event data_correction.c_none. */
    message_no_line_u_response_,            /**< Event message_no_line.u_response. */
    data_no_line_c_none_,                   /**< Event data_no_line.c_none. */
    component_LidarSensor_c_pL5GUHQARUVYQ_, /**< Event component_LidarSensor.c_pL5GUHQARUVYQ. */
    component_LidarSensor_c_pOTNR1GAQDAZE_, /**< Event component_LidarSensor.c_pOTNR1GAQDAZE. */
    message_scan_u_response_,               /**< Event message_scan.u_response. */
    data_scan_c_none_,                      /**< Event data_scan.c_none. */
    message_stop_u_response_,               /**< Event message_stop.u_response. */
    data_stop_c_none_,                      /**< Event data_stop.c_none. */
    message_continue_u_response_,           /**< Event message_continue.u_response. */
    data_continue_c_none_,                  /**< Event data_continue.c_none. */
    message_move_c_trigger_,                /**< Event message_move.c_trigger. */
    data_move_c_none_,                      /**< Event data_move.c_none. */
    data_move_c_p1FV6MKBBH2L7_,             /**< Event data_move.c_p1FV6MKBBH2L7. */
    message_halt_c_trigger_,                /**< Event message_halt.c_trigger. */
    data_halt_c_none_,                      /**< Event data_halt.c_none. */
    data_halt_c_pRJCB2BDWXUOW_,             /**< Event data_halt.c_pRJCB2BDWXUOW. */
};
typedef enum controllerEventEnum_ controller_Event_;

/** Names of all the events. */
extern const char *controller_event_names[];

/* Constants. */


/* Input variables. */
extern controllerEnum message_scan_i_response_; /**< Input variable "E message_scan.i_response". */

extern void controller_AssignInputVariables();

/* Declaration of internal functions. */


/* State variables (use for output only). */
extern RealType model_time; /**< Current model time. */
extern controllerEnum component_EmergencyStop_;                  /**< Discrete variable "E component_EmergencyStop". */
extern controllerEnum component_LidarSensor_v_current_distance_; /**< Discrete variable "E component_LidarSensor.v_current_distance". */
extern controllerEnum component_LidarSensor_;                    /**< Discrete variable "E component_LidarSensor". */
extern controllerEnum component_LineDetector_;                   /**< Discrete variable "E component_LineDetector". */
extern controllerEnum data_halt_;                                /**< Discrete variable "E data_halt". */
extern controllerEnum data_move_;                                /**< Discrete variable "E data_move". */

/* Algebraic and derivative functions (use for output only). */






/* Code entry points. */
void controller_EngineFirstStep(void);
void controller_EngineTimeStep(double delta);
BoolType controller_EnginePerformEvent(controller_Event_ event);

#if EVENT_OUTPUT
/**
 * External callback function reporting about the execution of an event.
 * @param event Event being executed.
 * @param pre If \c TRUE, event is about to be executed. If \c FALSE, event has been executed.
 * @note Function must be implemented externally.
 */
extern void controller_InfoEvent(controller_Event_ event, BoolType pre);
#endif

#if PRINT_OUTPUT
/**
 * External callback function to output the given text-line to the given filename.
 * @param text Text to print (does not have a EOL character).
 * @param fname Name of the file to print to.
 */
extern void controller_PrintOutput(const char *text, const char *fname);
#endif

#endif

