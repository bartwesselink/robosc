/* CIF to C translation of controller.cif
 * Generated file, DO NOT EDIT
 */

#include <stdio.h>
#include <stdlib.h>
#include "controller_engine.h"

#ifndef MAX_NUM_EVENTS
#define MAX_NUM_EVENTS 1000
#endif

/* What to do if a range error is found in an assignment? */
#ifdef KEEP_RUNNING
static inline void RangeErrorDetected(void) { /* Do nothing, error is already reported. */ }
#else
static inline void RangeErrorDetected(void) { exit(1); }
#endif

/* Type support code. */
int EnumTypePrint(controllerEnum value, char *dest, int start, int end) {
    int last = end - 1;
    const char *lit_name = enum_names[value];
    while (start < last && *lit_name) {
        dest[start++] = *lit_name;
        lit_name++;
    }
    dest[start] = '\0';
    return start;
}


/** Event names. */
const char *controller_event_names[] = {
    "initial-step",                             /**< Initial step. */
    "delay-step",                               /**< Delay step. */
    "tau",                                      /**< Tau step. */
    "message_scan_front.u_response",            /**< Event message_scan_front.u_response. */
    "data_scan_front.c_none",                   /**< Event data_scan_front.c_none. */
    "message_scan_left.u_response",             /**< Event message_scan_left.u_response. */
    "data_scan_left.c_none",                    /**< Event data_scan_left.c_none. */
    "message_scan_right.u_response",            /**< Event message_scan_right.u_response. */
    "data_scan_right.c_none",                   /**< Event data_scan_right.c_none. */
    "message_rotate_left.c_trigger",            /**< Event message_rotate_left.c_trigger. */
    "data_rotate_left.c_none",                  /**< Event data_rotate_left.c_none. */
    "message_rotate_right.c_trigger",           /**< Event message_rotate_right.c_trigger. */
    "data_rotate_right.c_none",                 /**< Event data_rotate_right.c_none. */
    "message_rotate_done.u_response",           /**< Event message_rotate_done.u_response. */
    "data_rotate_done.c_none",                  /**< Event data_rotate_done.c_none. */
    "component_ObjectDetector.c_pU6SJARKG2TD7", /**< Event component_ObjectDetector.c_pU6SJARKG2TD7. */
    "component_ObjectDetector.c_pEQ73M2PZ6GC9", /**< Event component_ObjectDetector.c_pEQ73M2PZ6GC9. */
    "message_object_count.u_response",          /**< Event message_object_count.u_response. */
    "data_object_count.c_none",                 /**< Event data_object_count.c_none. */
    "message_object_scan.u_response",           /**< Event message_object_scan.u_response. */
    "data_object_scan.c_none",                  /**< Event data_object_scan.c_none. */
    "message_move.c_trigger",                   /**< Event message_move.c_trigger. */
    "data_move.c_none",                         /**< Event data_move.c_none. */
    "data_move.c_p9ON15CK7WIV2",                /**< Event data_move.c_p9ON15CK7WIV2. */
    "message_halt.c_trigger",                   /**< Event message_halt.c_trigger. */
    "data_halt.c_none",                         /**< Event data_halt.c_none. */
    "data_halt.c_pZQW1X9M3P0X5",                /**< Event data_halt.c_pZQW1X9M3P0X5. */
    "message_stop.u_response",                  /**< Event message_stop.u_response. */
    "data_stop.c_none",                         /**< Event data_stop.c_none. */
    "message_continue.u_response",              /**< Event message_continue.u_response. */
    "data_continue.c_none",                     /**< Event data_continue.c_none. */
};

/** Enumeration names. */
const char *enum_names[] = {
    "awaiting_command",
    "data_pLV2V0NPQH35K",
    "data_pO6JO6QW5TBOU",
    "executing",
    "in_service",
    "no_object",
    "none",
    "object_found",
    "safe_front",
    "safe_left",
    "safe_right",
    "stop_sign",
    "stopped",
    "unsafe_front",
    "unsafe_left",
    "unsafe_right",
};

/* Constants. */


/* Functions. */


/* Input variables. */
controllerEnum message_scan_front_i_response_;  /**< Input variable "E message_scan_front.i_response". */
controllerEnum message_scan_left_i_response_;   /**< Input variable "E message_scan_left.i_response". */
controllerEnum message_scan_right_i_response_;  /**< Input variable "E message_scan_right.i_response". */
IntType message_object_count_i_response_count_; /**< Input variable "int[0..1] message_object_count.i_response_count". */
controllerEnum message_object_scan_i_response_; /**< Input variable "E message_object_scan.i_response". */

/* State variables. */
controllerEnum component_EmergencyStop_;                   /**< Discrete variable "E component_EmergencyStop". */
controllerEnum component_LidarScanner_v_front_;            /**< Discrete variable "E component_LidarScanner.v_front". */
controllerEnum component_LidarScanner_v_left_;             /**< Discrete variable "E component_LidarScanner.v_left". */
controllerEnum component_LidarScanner_v_right_;            /**< Discrete variable "E component_LidarScanner.v_right". */
BoolType component_LidarScanner_v_has_front_;              /**< Discrete variable "bool component_LidarScanner.v_has_front". */
IntType component_ObjectDetector_v_scanned_object_count_;  /**< Discrete variable "int[0..1] component_ObjectDetector.v_scanned_object_count". */
controllerEnum component_ObjectDetector_v_scanned_object_; /**< Discrete variable "E component_ObjectDetector.v_scanned_object". */
controllerEnum component_ObjectDetector_;                  /**< Discrete variable "E component_ObjectDetector". */
controllerEnum component_Rotator_;                         /**< Discrete variable "E component_Rotator". */
controllerEnum data_halt_;                                 /**< Discrete variable "E data_halt". */
controllerEnum data_move_;                                 /**< Discrete variable "E data_move". */

RealType model_time; /**< Current model time. */

/** Initialize constants. */
static void InitConstants(void) {

}

/** Print function. */
#if PRINT_OUTPUT
static void PrintOutput(controller_Event_ event, BoolType pre) {
}
#endif

/* Event execution code. */

/**
 * Execute code for event "component_ObjectDetector.c_pEQ73M2PZ6GC9".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent0(void) {
    BoolType guard = ((component_ObjectDetector_) == (_controller_object_found)) && ((component_ObjectDetector_v_scanned_object_count_) == (0));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(component_ObjectDetector_c_pEQ73M2PZ6GC9_, TRUE);
    #endif

    component_ObjectDetector_ = _controller_no_object;

    #if EVENT_OUTPUT
        controller_InfoEvent(component_ObjectDetector_c_pEQ73M2PZ6GC9_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "component_ObjectDetector.c_pU6SJARKG2TD7".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent1(void) {
    BoolType guard = ((component_ObjectDetector_) == (_controller_no_object)) && (((component_ObjectDetector_v_scanned_object_count_) > (0)) && ((component_ObjectDetector_v_scanned_object_) == (_controller_stop_sign)));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(component_ObjectDetector_c_pU6SJARKG2TD7_, TRUE);
    #endif

    component_ObjectDetector_ = _controller_object_found;

    #if EVENT_OUTPUT
        controller_InfoEvent(component_ObjectDetector_c_pU6SJARKG2TD7_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_continue.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent2(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_continue_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_continue_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_halt.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent3(void) {
    BoolType guard = FALSE;
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_halt_c_none_, TRUE);
    #endif

    if (FALSE) {
        data_halt_ = _controller_none;
    } else if (FALSE) {
        data_halt_ = _controller_none;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_halt_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_halt.c_pZQW1X9M3P0X5".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent4(void) {
    BoolType guard = ((data_halt_) == (_controller_none)) || ((data_halt_) == (_controller_data_pLV2V0NPQH35K));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_halt_c_pZQW1X9M3P0X5_, TRUE);
    #endif

    if ((data_halt_) == (_controller_none)) {
        data_halt_ = _controller_data_pLV2V0NPQH35K;
    } else if ((data_halt_) == (_controller_data_pLV2V0NPQH35K)) {
        data_halt_ = _controller_data_pLV2V0NPQH35K;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_halt_c_pZQW1X9M3P0X5_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_move.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent5(void) {
    BoolType guard = FALSE;
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_none_, TRUE);
    #endif

    if (FALSE) {
        data_move_ = _controller_none;
    } else if (FALSE) {
        data_move_ = _controller_none;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_move.c_p9ON15CK7WIV2".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent6(void) {
    BoolType guard = ((data_move_) == (_controller_none)) || ((data_move_) == (_controller_data_pO6JO6QW5TBOU));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_p9ON15CK7WIV2_, TRUE);
    #endif

    if ((data_move_) == (_controller_none)) {
        data_move_ = _controller_data_pO6JO6QW5TBOU;
    } else if ((data_move_) == (_controller_data_pO6JO6QW5TBOU)) {
        data_move_ = _controller_data_pO6JO6QW5TBOU;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_p9ON15CK7WIV2_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_object_count.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent7(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_object_count_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_object_count_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_object_scan.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent8(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_object_scan_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_object_scan_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_rotate_done.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent9(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_rotate_done_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_rotate_done_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_rotate_left.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent10(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_rotate_left_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_rotate_left_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_rotate_right.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent11(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_rotate_right_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_rotate_right_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_scan_front.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent12(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_scan_front_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_scan_front_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_scan_left.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent13(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_scan_left_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_scan_left_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_scan_right.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent14(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_scan_right_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_scan_right_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_stop.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent15(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_stop_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_stop_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_continue.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent16(void) {
    BoolType guard = (((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && ((((component_ObjectDetector_) == (_controller_no_object)) || ((component_ObjectDetector_) == (_controller_object_found))) && (((component_Rotator_) == (_controller_awaiting_command)) || ((component_Rotator_) == (_controller_executing))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_continue_u_response_, TRUE);
    #endif

    if ((component_EmergencyStop_) == (_controller_in_service)) {
        component_EmergencyStop_ = _controller_in_service;
    } else if ((component_EmergencyStop_) == (_controller_stopped)) {
        component_EmergencyStop_ = _controller_in_service;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(message_continue_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_halt.c_trigger".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent17(void) {
    BoolType guard = ((((component_LidarScanner_v_left_) == (_controller_unsafe_left)) && ((component_LidarScanner_v_right_) == (_controller_unsafe_right))) && ((component_LidarScanner_v_front_) == (_controller_unsafe_front))) || (((component_ObjectDetector_) == (_controller_object_found)) || ((component_EmergencyStop_) == (_controller_stopped)));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_halt_c_trigger_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(message_halt_c_trigger_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_move.c_trigger".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent18(void) {
    BoolType guard = ((component_LidarScanner_v_front_) == (_controller_safe_front)) && (((component_Rotator_) == (_controller_awaiting_command)) && (((component_EmergencyStop_) == (_controller_in_service)) && ((component_ObjectDetector_ != _controller_object_found))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_move_c_trigger_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(message_move_c_trigger_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_object_count.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent19(void) {
    BoolType guard = (((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && ((((component_ObjectDetector_) == (_controller_no_object)) || ((component_ObjectDetector_) == (_controller_object_found))) && (((component_Rotator_) == (_controller_awaiting_command)) || ((component_Rotator_) == (_controller_executing))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_object_count_u_response_, TRUE);
    #endif

    if ((component_ObjectDetector_) == (_controller_no_object)) {
        component_ObjectDetector_v_scanned_object_count_ = message_object_count_i_response_count_;
    } else if ((component_ObjectDetector_) == (_controller_object_found)) {
        component_ObjectDetector_v_scanned_object_count_ = message_object_count_i_response_count_;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(message_object_count_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_object_scan.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent20(void) {
    BoolType guard = (((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && ((((component_ObjectDetector_) == (_controller_no_object)) || ((component_ObjectDetector_) == (_controller_object_found))) && (((component_Rotator_) == (_controller_awaiting_command)) || ((component_Rotator_) == (_controller_executing))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_object_scan_u_response_, TRUE);
    #endif

    if ((component_ObjectDetector_) == (_controller_no_object)) {
        component_ObjectDetector_v_scanned_object_ = message_object_scan_i_response_;
    } else if ((component_ObjectDetector_) == (_controller_object_found)) {
        component_ObjectDetector_v_scanned_object_ = message_object_scan_i_response_;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(message_object_scan_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_rotate_done.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent21(void) {
    BoolType guard = (((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && ((((component_ObjectDetector_) == (_controller_no_object)) || ((component_ObjectDetector_) == (_controller_object_found))) && (((component_Rotator_) == (_controller_awaiting_command)) || ((component_Rotator_) == (_controller_executing))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_rotate_done_u_response_, TRUE);
    #endif

    if ((component_Rotator_) == (_controller_awaiting_command)) {
        component_Rotator_ = _controller_awaiting_command;
    } else if ((component_Rotator_) == (_controller_executing)) {
        component_Rotator_ = _controller_awaiting_command;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(message_rotate_done_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_rotate_left.c_trigger".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent22(void) {
    BoolType guard = ((component_Rotator_) == (_controller_awaiting_command)) && ((((component_LidarScanner_v_left_) == (_controller_safe_left)) && (((component_ObjectDetector_ != _controller_object_found)) && ((component_EmergencyStop_) == (_controller_in_service)))) && (((component_Rotator_) == (_controller_awaiting_command)) && (((component_LidarScanner_v_front_) == (_controller_unsafe_front)) && (component_LidarScanner_v_has_front_))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_rotate_left_c_trigger_, TRUE);
    #endif

    component_Rotator_ = _controller_executing;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_rotate_left_c_trigger_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_rotate_right.c_trigger".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent23(void) {
    BoolType guard = ((component_Rotator_) == (_controller_awaiting_command)) && ((((component_LidarScanner_v_right_) == (_controller_safe_right)) && (((component_ObjectDetector_ != _controller_object_found)) && ((component_EmergencyStop_) == (_controller_in_service)))) && (((component_Rotator_) == (_controller_awaiting_command)) && (((component_LidarScanner_v_front_) == (_controller_unsafe_front)) && (component_LidarScanner_v_has_front_))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_rotate_right_c_trigger_, TRUE);
    #endif

    component_Rotator_ = _controller_executing;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_rotate_right_c_trigger_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_scan_front.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent24(void) {
    BoolType guard = (((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && ((((component_ObjectDetector_) == (_controller_no_object)) || ((component_ObjectDetector_) == (_controller_object_found))) && (((component_Rotator_) == (_controller_awaiting_command)) || ((component_Rotator_) == (_controller_executing))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_scan_front_u_response_, TRUE);
    #endif

    component_LidarScanner_v_front_ = message_scan_front_i_response_;
    component_LidarScanner_v_has_front_ = TRUE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_scan_front_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_scan_left.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent25(void) {
    BoolType guard = (((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && ((((component_ObjectDetector_) == (_controller_no_object)) || ((component_ObjectDetector_) == (_controller_object_found))) && (((component_Rotator_) == (_controller_awaiting_command)) || ((component_Rotator_) == (_controller_executing))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_scan_left_u_response_, TRUE);
    #endif

    component_LidarScanner_v_left_ = message_scan_left_i_response_;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_scan_left_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_scan_right.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent26(void) {
    BoolType guard = (((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && ((((component_ObjectDetector_) == (_controller_no_object)) || ((component_ObjectDetector_) == (_controller_object_found))) && (((component_Rotator_) == (_controller_awaiting_command)) || ((component_Rotator_) == (_controller_executing))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_scan_right_u_response_, TRUE);
    #endif

    component_LidarScanner_v_right_ = message_scan_right_i_response_;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_scan_right_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_stop.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent27(void) {
    BoolType guard = (((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && ((((component_ObjectDetector_) == (_controller_no_object)) || ((component_ObjectDetector_) == (_controller_object_found))) && (((component_Rotator_) == (_controller_awaiting_command)) || ((component_Rotator_) == (_controller_executing))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_stop_u_response_, TRUE);
    #endif

    if ((component_EmergencyStop_) == (_controller_in_service)) {
        component_EmergencyStop_ = _controller_stopped;
    } else if ((component_EmergencyStop_) == (_controller_stopped)) {
        component_EmergencyStop_ = _controller_stopped;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(message_stop_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Normalize and check the new value of a continuous variable after an update.
 * @param new_value Unnormalized new value of the continuous variable.
 * @param var_name Name of the continuous variable in the CIF model.
 * @return The normalized new value of the continuous variable.
 */
static inline RealType UpdateContValue(RealType new_value, const char *var_name) {
    if (isfinite(new_value)) {
        return (new_value == -0.0) ? 0.0 : new_value;
    }

    const char *err_type;
    if (isnan(new_value)) {
        err_type = "NaN";
    } else if (new_value > 0) {
        err_type = "+inf";
    } else {
        err_type = "-inf";
    }
    fprintf(stderr, "Continuous variable \"%s\" has become %s.\n", var_name, err_type);

#ifdef KEEP_RUNNING
    return 0.0;
#else
    exit(1);
#endif
}

/** Repeatedly perform discrete event steps, until no progress can be made any more. */
static void PerformEvents(void) {
    int count = 0;
    for (;;) {
        count++;
        if (count > MAX_NUM_EVENTS) { /* 'Infinite' loop detection. */
            fprintf(stderr, "Warning: Quitting after performing %d events, infinite loop?\n", count);
            break;
        }

        if (execEvent0()) continue;  /* (Try to) perform event "component_ObjectDetector.c_pEQ73M2PZ6GC9". */
        if (execEvent1()) continue;  /* (Try to) perform event "component_ObjectDetector.c_pU6SJARKG2TD7". */
        if (execEvent2()) continue;  /* (Try to) perform event "data_continue.c_none". */
        if (execEvent3()) continue;  /* (Try to) perform event "data_halt.c_none". */
        if (execEvent4()) continue;  /* (Try to) perform event "data_halt.c_pZQW1X9M3P0X5". */
        if (execEvent5()) continue;  /* (Try to) perform event "data_move.c_none". */
        if (execEvent6()) continue;  /* (Try to) perform event "data_move.c_p9ON15CK7WIV2". */
        if (execEvent7()) continue;  /* (Try to) perform event "data_object_count.c_none". */
        if (execEvent8()) continue;  /* (Try to) perform event "data_object_scan.c_none". */
        if (execEvent9()) continue;  /* (Try to) perform event "data_rotate_done.c_none". */
        if (execEvent10()) continue;  /* (Try to) perform event "data_rotate_left.c_none". */
        if (execEvent11()) continue;  /* (Try to) perform event "data_rotate_right.c_none". */
        if (execEvent12()) continue;  /* (Try to) perform event "data_scan_front.c_none". */
        if (execEvent13()) continue;  /* (Try to) perform event "data_scan_left.c_none". */
        if (execEvent14()) continue;  /* (Try to) perform event "data_scan_right.c_none". */
        if (execEvent15()) continue;  /* (Try to) perform event "data_stop.c_none". */
        if (execEvent17()) continue;  /* (Try to) perform event "message_halt.c_trigger". */
        if (execEvent18()) continue;  /* (Try to) perform event "message_move.c_trigger". */
        if (execEvent22()) continue;  /* (Try to) perform event "message_rotate_left.c_trigger". */
        if (execEvent23()) continue;  /* (Try to) perform event "message_rotate_right.c_trigger". */
        break; /* No event fired, done with discrete steps. */
    }
}

/** First model call, initializing, and performing discrete events before the first time step. */
void controller_EngineFirstStep(void) {
    InitConstants();

    model_time = 0.0;
    controller_AssignInputVariables();
    component_EmergencyStop_ = _controller_in_service;
    component_LidarScanner_v_front_ = _controller_safe_front;
    component_LidarScanner_v_left_ = _controller_unsafe_left;
    component_LidarScanner_v_right_ = _controller_unsafe_right;
    component_LidarScanner_v_has_front_ = FALSE;
    component_ObjectDetector_v_scanned_object_count_ = 0;
    component_ObjectDetector_v_scanned_object_ = _controller_stop_sign;
    component_ObjectDetector_ = _controller_no_object;
    component_Rotator_ = _controller_awaiting_command;
    data_halt_ = _controller_none;
    data_move_ = _controller_none;

    #if PRINT_OUTPUT
        /* pre-initial and post-initial prints. */
        PrintOutput(EVT_INITIAL_, TRUE);
        PrintOutput(EVT_INITIAL_, FALSE);
    #endif

    PerformEvents();

    #if PRINT_OUTPUT
        /* pre-timestep print. */
        PrintOutput(EVT_DELAY_, TRUE);
    #endif
}

/**
 * Engine takes a time step of length \a delta.
 * @param delta Length of the time step.
 */
void controller_EngineTimeStep(double delta) {
    controller_AssignInputVariables();

    /* Update continuous variables. */
    if (delta > 0.0) {

        model_time += delta;
    }

    #if PRINT_OUTPUT
        /* post-timestep print. */
        PrintOutput(EVT_DELAY_, FALSE);
    #endif

    PerformEvents();

    #if PRINT_OUTPUT
        /* pre-timestep print. */
        PrintOutput(EVT_DELAY_, TRUE);
    #endif
}

/**
  * Engine (attempts) to perform a single event.
  * @param event The event to perform.
  * @return Whether the specified event could be executed.
  */
BoolType controller_EnginePerformEvent(controller_Event_ event) {
    switch (event) {
        case component_ObjectDetector_c_pEQ73M2PZ6GC9_:
            return execEvent0();
        case component_ObjectDetector_c_pU6SJARKG2TD7_:
            return execEvent1();
        case data_continue_c_none_:
            return execEvent2();
        case data_halt_c_none_:
            return execEvent3();
        case data_halt_c_pZQW1X9M3P0X5_:
            return execEvent4();
        case data_move_c_none_:
            return execEvent5();
        case data_move_c_p9ON15CK7WIV2_:
            return execEvent6();
        case data_object_count_c_none_:
            return execEvent7();
        case data_object_scan_c_none_:
            return execEvent8();
        case data_rotate_done_c_none_:
            return execEvent9();
        case data_rotate_left_c_none_:
            return execEvent10();
        case data_rotate_right_c_none_:
            return execEvent11();
        case data_scan_front_c_none_:
            return execEvent12();
        case data_scan_left_c_none_:
            return execEvent13();
        case data_scan_right_c_none_:
            return execEvent14();
        case data_stop_c_none_:
            return execEvent15();
        case message_continue_u_response_:
            return execEvent16();
        case message_halt_c_trigger_:
            return execEvent17();
        case message_move_c_trigger_:
            return execEvent18();
        case message_object_count_u_response_:
            return execEvent19();
        case message_object_scan_u_response_:
            return execEvent20();
        case message_rotate_done_u_response_:
            return execEvent21();
        case message_rotate_left_c_trigger_:
            return execEvent22();
        case message_rotate_right_c_trigger_:
            return execEvent23();
        case message_scan_front_u_response_:
            return execEvent24();
        case message_scan_left_u_response_:
            return execEvent25();
        case message_scan_right_u_response_:
            return execEvent26();
        case message_stop_u_response_:
            return execEvent27();
        default:
            return FALSE;
    }
}
