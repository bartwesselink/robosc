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
    _controller_data_p5J0MYKIFPJAO,
    _controller_data_pC7OJ0RYRXLPS,
    _controller_data_pIGGZFZSZR4F9,
    _controller_data_pSMSKNS3BMH48,
    _controller_detected,
    _controller_free,
    _controller_in_service,
    _controller_initializing,
    _controller_none,
    _controller_person,
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
    message_scan_u_response_,           /**< Event message_scan.u_response. */
    message_bounding_boxes_u_response_, /**< Event message_bounding_boxes.u_response. */
    message_stop_u_response_,           /**< Event message_stop.u_response. */
    message_continue_u_response_,       /**< Event message_continue.u_response. */
    message_move_c_trigger_,            /**< Event message_move.c_trigger. */
    data_move_c_p0LY3EZ9AGQNL_,         /**< Event data_move.c_p0LY3EZ9AGQNL. */
    data_move_c_pUAQU3XG1WYHR_,         /**< Event data_move.c_pUAQU3XG1WYHR. */
    data_move_c_pUSGSRSK1JLC3_,         /**< Event data_move.c_pUSGSRSK1JLC3. */
    message_halt_c_trigger_,            /**< Event message_halt.c_trigger. */
    data_halt_c_pWJL900TG040Y_,         /**< Event data_halt.c_pWJL900TG040Y. */
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
extern controllerEnum component_EmergencyStop_;      /**< Discrete variable "E component_EmergencyStop". */
extern controllerEnum component_Scanner_v_distance_; /**< Discrete variable "E component_Scanner.v_distance". */
extern controllerEnum component_YoloxDetection_;     /**< Discrete variable "E component_YoloxDetection". */
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

