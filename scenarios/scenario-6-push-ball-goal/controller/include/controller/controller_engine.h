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
    _controller_data_p3ZGYS6KSU0XF,
    _controller_data_pE6GB07T4JXOC,
    _controller_data_pIPSC26A303FW,
    _controller_data_pK45EP742XFHY,
    _controller_data_pL7787Q1V71VA,
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
    data_move_c_p1T2KKZZJNFA3_,           /**< Event data_move.c_p1T2KKZZJNFA3. */
    data_move_c_p550UT5R5J36H_,           /**< Event data_move.c_p550UT5R5J36H. */
    data_move_c_p0O96RQ4VS3ZF_,           /**< Event data_move.c_p0O96RQ4VS3ZF. */
    data_move_c_pNFY8NNNM57ID_,           /**< Event data_move.c_pNFY8NNNM57ID. */
    message_halt_c_trigger_,              /**< Event message_halt.c_trigger. */
    data_halt_c_pOCNJONKQIKWU_,           /**< Event data_halt.c_pOCNJONKQIKWU. */
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

