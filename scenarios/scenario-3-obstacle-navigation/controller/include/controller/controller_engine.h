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
    _controller_awaiting_point,
    _controller_data_pKRN56FXDJPW6,
    _controller_error,
    _controller_executing,
    _controller_has_point,
    _controller_idle,
    _controller_in_service,
    _controller_no_initial_pose,
    _controller_none,
    _controller_ready,
    _controller_stopped,
};
typedef enum Enumcontroller_ controllerEnum;

extern const char *enum_names[];
extern int EnumTypePrint(controllerEnum value, char *dest, int start, int end);


/* Event declarations. */
enum controllerEventEnum_ {
    EVT_INITIAL_,                     /**< Initial step. */
    EVT_DELAY_,                       /**< Delay step. */
    EVT_TAU_,                         /**< Tau step. */
    message_point_u_response_,        /**< Event message_point.u_response. */
    data_point_c_none_,               /**< Event data_point.c_none. */
    message_initial_pose_u_response_, /**< Event message_initial_pose.u_response. */
    data_initial_pose_c_none_,        /**< Event data_initial_pose.c_none. */
    action_navigate_c_trigger_,       /**< Event action_navigate.c_trigger. */
    action_navigate_c_reset_,         /**< Event action_navigate.c_reset. */
    action_navigate_c_cancel_,        /**< Event action_navigate.c_cancel. */
    action_navigate_u_feedback_,      /**< Event action_navigate.u_feedback. */
    action_navigate_u_response_,      /**< Event action_navigate.u_response. */
    action_navigate_u_error_,         /**< Event action_navigate.u_error. */
    data_navigate_c_none_,            /**< Event data_navigate.c_none. */
    data_navigate_c_pPOS5YMEJUPC6_,   /**< Event data_navigate.c_pPOS5YMEJUPC6. */
    message_stop_u_response_,         /**< Event message_stop.u_response. */
    data_stop_c_none_,                /**< Event data_stop.c_none. */
    message_continue_u_response_,     /**< Event message_continue.u_response. */
    data_continue_c_none_,            /**< Event data_continue.c_none. */
};
typedef enum controllerEventEnum_ controller_Event_;

/** Names of all the events. */
extern const char *controller_event_names[];

/* Constants. */


/* Input variables. */




/* Declaration of internal functions. */


/* State variables (use for output only). */
extern RealType model_time; /**< Current model time. */
extern controllerEnum action_navigate_;         /**< Discrete variable "E action_navigate". */
extern controllerEnum component_EmergencyStop_; /**< Discrete variable "E component_EmergencyStop". */
extern controllerEnum component_Nav2_;          /**< Discrete variable "E component_Nav2". */
extern controllerEnum data_navigate_;           /**< Discrete variable "E data_navigate". */

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

