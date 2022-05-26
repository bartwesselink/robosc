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
    _controller_data_p4WFYGT86LJK9,
    _controller_data_p641V0HTAP6A3,
    _controller_data_pJ7PR7U36O9D9,
    _controller_data_pR8MHMD5FPBMS,
    _controller_data_pX11JBUY6PUJ1,
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
    message_ball_correction_u_response_,  /**< Event message_ball_correction.u_response. */
    message_no_ball_u_response_,          /**< Event message_no_ball.u_response. */
    message_needs_ajustment_u_response_,  /**< Event message_needs_ajustment.u_response. */
    message_no_adjustment_u_response_,    /**< Event message_no_adjustment.u_response. */
    message_ball_front_check_u_response_, /**< Event message_ball_front_check.u_response. */
    message_goal_correction_u_response_,  /**< Event message_goal_correction.u_response. */
    message_no_goal_u_response_,          /**< Event message_no_goal.u_response. */
    message_stop_u_response_,             /**< Event message_stop.u_response. */
    message_continue_u_response_,         /**< Event message_continue.u_response. */
    message_move_c_trigger_,              /**< Event message_move.c_trigger. */
    data_move_c_pP7JD423L253E_,           /**< Event data_move.c_pP7JD423L253E. */
    data_move_c_pP4MR7QKY1OJY_,           /**< Event data_move.c_pP4MR7QKY1OJY. */
    data_move_c_pBZQBH3JQ63HW_,           /**< Event data_move.c_pBZQBH3JQ63HW. */
    data_move_c_pQ39O1MCHV35W_,           /**< Event data_move.c_pQ39O1MCHV35W. */
    message_halt_c_trigger_,              /**< Event message_halt.c_trigger. */
    data_halt_c_pM4VG4RLT77T1_,           /**< Event data_halt.c_pM4VG4RLT77T1. */
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

