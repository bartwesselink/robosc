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
    _controller_awaiting,
    _controller_data_p1POFUPHF3J9D,
    _controller_data_pE12IJL1XOLG7,
    _controller_in_service,
    _controller_line_found,
    _controller_no_line,
    _controller_none,
    _controller_stopped,
};
typedef enum Enumcontroller_ controllerEnum;

extern const char *enum_names[];
extern int EnumTypePrint(controllerEnum value, char *dest, int start, int end);


/* Event declarations. */
enum controllerEventEnum_ {
    EVT_INITIAL_,                       /**< Initial step. */
    EVT_DELAY_,                         /**< Delay step. */
    EVT_TAU_,                           /**< Tau step. */
    message_correction_u_response_,     /**< Event message_correction.u_response. */
    message_no_line_u_response_,        /**< Event message_no_line.u_response. */
    message_stop_u_response_,           /**< Event message_stop.u_response. */
    message_continue_u_response_,       /**< Event message_continue.u_response. */
    message_move_c_trigger_,    /**< Event message_move.c_trigger. */
    data_move_c_pIELFJLWO5ADT_, /**< Event data_move.c_pIELFJLWO5ADT. */
    message_halt_c_trigger_,            /**< Event message_halt.c_trigger. */
    data_halt_c_pUOCHN7MH2YLV_,         /**< Event data_halt.c_pUOCHN7MH2YLV. */
};
typedef enum controllerEventEnum_ controller_Event_;

/** Names of all the events. */
extern const char *controller_event_names[];

/* Constants. */


/* Input variables. */




/* Declaration of internal functions. */


/* State variables (use for output only). */
extern RealType model_time; /**< Current model time. */
extern controllerEnum component_EmergencyStop_; /**< Discrete variable "E component_EmergencyStop". */
extern controllerEnum component_LineDetector_;  /**< Discrete variable "E component_LineDetector". */
extern controllerEnum data_halt_;               /**< Discrete variable "E data_halt". */
extern controllerEnum data_move_;       /**< Discrete variable "E data_move". */

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

