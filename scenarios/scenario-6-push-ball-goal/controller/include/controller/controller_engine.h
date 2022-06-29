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
    _controller_adjusting,
    _controller_awaiting,
    _controller_ball_found,
    _controller_ball_in_front,
    _controller_data_p3FWU5LJD83KP,
    _controller_data_pT6I8XRJ5MOT3,
    _controller_data_pWKTMPZDWUE17,
    _controller_data_pZQUQLRDM9OJ6,
    _controller_data_pZQUXVJGJO0YV,
    _controller_free,
    _controller_goal_found,
    _controller_in_service,
    _controller_no_ball,
    _controller_no_goal,
    _controller_none,
    _controller_obstructed,
    _controller_stopped,
};
typedef enum Enumcontroller_ controllerEnum;

extern const char *enum_names[];
extern int EnumTypePrint(controllerEnum value, char *dest, int start, int end);


/* Event declarations. */
enum controllerEventEnum_ {
    EVT_INITIAL_,                         /**< Initial step. */
    EVT_DELAY_,                           /**< Delay step. */
    EVT_TAU_,                             /**< Tau step. */
    message_scan_u_response_,             /**< Event message_scan.u_response. */
    data_scan_c_none_,                    /**< Event data_scan.c_none. */
    message_ball_correction_u_response_,  /**< Event message_ball_correction.u_response. */
    data_ball_correction_c_none_,         /**< Event data_ball_correction.c_none. */
    message_no_ball_u_response_,          /**< Event message_no_ball.u_response. */
    data_no_ball_c_none_,                 /**< Event data_no_ball.c_none. */
    message_needs_ajustment_u_response_,  /**< Event message_needs_ajustment.u_response. */
    data_needs_ajustment_c_none_,         /**< Event data_needs_ajustment.c_none. */
    message_no_adjustment_u_response_,    /**< Event message_no_adjustment.u_response. */
    data_no_adjustment_c_none_,           /**< Event data_no_adjustment.c_none. */
    message_ball_front_check_u_response_, /**< Event message_ball_front_check.u_response. */
    data_ball_front_check_c_none_,        /**< Event data_ball_front_check.c_none. */
    message_goal_correction_u_response_,  /**< Event message_goal_correction.u_response. */
    data_goal_correction_c_none_,         /**< Event data_goal_correction.c_none. */
    message_no_goal_u_response_,          /**< Event message_no_goal.u_response. */
    data_no_goal_c_none_,                 /**< Event data_no_goal.c_none. */
    message_stop_u_response_,             /**< Event message_stop.u_response. */
    data_stop_c_none_,                    /**< Event data_stop.c_none. */
    message_continue_u_response_,         /**< Event message_continue.u_response. */
    data_continue_c_none_,                /**< Event data_continue.c_none. */
    message_move_c_trigger_,              /**< Event message_move.c_trigger. */
    data_move_c_none_,                    /**< Event data_move.c_none. */
    data_move_c_pNLRTW551QX6M_,           /**< Event data_move.c_pNLRTW551QX6M. */
    data_move_c_pLQBN5IDMMNA0_,           /**< Event data_move.c_pLQBN5IDMMNA0. */
    data_move_c_pZ6VVX7FHA9UC_,           /**< Event data_move.c_pZ6VVX7FHA9UC. */
    data_move_c_pHKVHWHN6EUM1_,           /**< Event data_move.c_pHKVHWHN6EUM1. */
    message_halt_c_trigger_,              /**< Event message_halt.c_trigger. */
    data_halt_c_none_,                    /**< Event data_halt.c_none. */
    data_halt_c_p7A4YL2QMRJC0_,           /**< Event data_halt.c_p7A4YL2QMRJC0. */
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
extern controllerEnum component_BallDetector_;       /**< Discrete variable "E component_BallDetector". */
extern controllerEnum component_EmergencyStop_;      /**< Discrete variable "E component_EmergencyStop". */
extern controllerEnum component_GoalDetector_;       /**< Discrete variable "E component_GoalDetector". */
extern controllerEnum component_Scanner_v_distance_; /**< Discrete variable "E component_Scanner.v_distance". */
extern controllerEnum data_halt_;                    /**< Discrete variable "E data_halt". */
extern controllerEnum data_move_;                    /**< Discrete variable "E data_move". */

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

