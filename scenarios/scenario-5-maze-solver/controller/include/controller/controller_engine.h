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
    _controller_data_p0L2DOVIZA0HQ,
    _controller_data_p37TDJ4O4Z9W2,
    _controller_data_p56AAXCY6NDQE,
    _controller_data_p6Z88MTB24NZY,
    _controller_in_service,
    _controller_no_wall_diag_right,
    _controller_no_wall_front,
    _controller_no_wall_right,
    _controller_none,
    _controller_ready,
    _controller_stopped,
    _controller_turning,
    _controller_wall_diag_right,
    _controller_wall_front,
    _controller_wall_right,
};
typedef enum Enumcontroller_ controllerEnum;

extern const char *enum_names[];
extern int EnumTypePrint(controllerEnum value, char *dest, int start, int end);


/* Event declarations. */
enum controllerEventEnum_ {
    EVT_INITIAL_,                        /**< Initial step. */
    EVT_DELAY_,                          /**< Delay step. */
    EVT_TAU_,                            /**< Tau step. */
    message_scan_right_u_response_,      /**< Event message_scan_right.u_response. */
    data_scan_right_c_none_,             /**< Event data_scan_right.c_none. */
    message_scan_front_u_response_,      /**< Event message_scan_front.u_response. */
    data_scan_front_c_none_,             /**< Event data_scan_front.c_none. */
    message_scan_diag_right_u_response_, /**< Event message_scan_diag_right.u_response. */
    data_scan_diag_right_c_none_,        /**< Event data_scan_diag_right.c_none. */
    message_movement_c_trigger_,         /**< Event message_movement.c_trigger. */
    data_movement_c_none_,               /**< Event data_movement.c_none. */
    data_movement_c_pNVA8LDXZ1W47_,      /**< Event data_movement.c_pNVA8LDXZ1W47. */
    message_halt_c_trigger_,             /**< Event message_halt.c_trigger. */
    data_halt_c_none_,                   /**< Event data_halt.c_none. */
    data_halt_c_p16BE8S338R6N_,          /**< Event data_halt.c_p16BE8S338R6N. */
    message_turn_left_c_trigger_,        /**< Event message_turn_left.c_trigger. */
    data_turn_left_c_none_,              /**< Event data_turn_left.c_none. */
    data_turn_left_c_pE8ZDXWKF7DDQ_,     /**< Event data_turn_left.c_pE8ZDXWKF7DDQ. */
    message_turn_right_c_trigger_,       /**< Event message_turn_right.c_trigger. */
    data_turn_right_c_none_,             /**< Event data_turn_right.c_none. */
    data_turn_right_c_pA9IRG3XACGBX_,    /**< Event data_turn_right.c_pA9IRG3XACGBX. */
    message_rotate_done_u_response_,     /**< Event message_rotate_done.u_response. */
    data_rotate_done_c_none_,            /**< Event data_rotate_done.c_none. */
    message_stop_u_response_,            /**< Event message_stop.u_response. */
    data_stop_c_none_,                   /**< Event data_stop.c_none. */
    message_continue_u_response_,        /**< Event message_continue.u_response. */
    data_continue_c_none_,               /**< Event data_continue.c_none. */
};
typedef enum controllerEventEnum_ controller_Event_;

/** Names of all the events. */
extern const char *controller_event_names[];

/* Constants. */


/* Input variables. */
extern controllerEnum message_scan_right_i_response_;      /**< Input variable "E message_scan_right.i_response". */
extern controllerEnum message_scan_front_i_response_;      /**< Input variable "E message_scan_front.i_response". */
extern controllerEnum message_scan_diag_right_i_response_; /**< Input variable "E message_scan_diag_right.i_response". */

extern void controller_AssignInputVariables();

/* Declaration of internal functions. */


/* State variables (use for output only). */
extern RealType model_time; /**< Current model time. */
extern controllerEnum component_Distance_v_right_;      /**< Discrete variable "E component_Distance.v_right". */
extern controllerEnum component_Distance_v_front_;      /**< Discrete variable "E component_Distance.v_front". */
extern controllerEnum component_Distance_v_diag_right_; /**< Discrete variable "E component_Distance.v_diag_right". */
extern controllerEnum component_EmergencyStop_;         /**< Discrete variable "E component_EmergencyStop". */
extern controllerEnum component_Platform_;              /**< Discrete variable "E component_Platform". */
extern controllerEnum data_halt_;                       /**< Discrete variable "E data_halt". */
extern controllerEnum data_movement_;                   /**< Discrete variable "E data_movement". */
extern controllerEnum data_turn_left_;                  /**< Discrete variable "E data_turn_left". */
extern controllerEnum data_turn_right_;                 /**< Discrete variable "E data_turn_right". */

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

