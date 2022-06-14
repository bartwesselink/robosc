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
    "initial-step",                        /**< Initial step. */
    "delay-step",                          /**< Delay step. */
    "tau",                                 /**< Tau step. */
    "message_scan.u_response",             /**< Event message_scan.u_response. */
    "data_scan.c_none",                    /**< Event data_scan.c_none. */
    "message_ball_correction.u_response",  /**< Event message_ball_correction.u_response. */
    "data_ball_correction.c_none",         /**< Event data_ball_correction.c_none. */
    "message_no_ball.u_response",          /**< Event message_no_ball.u_response. */
    "data_no_ball.c_none",                 /**< Event data_no_ball.c_none. */
    "message_needs_ajustment.u_response",  /**< Event message_needs_ajustment.u_response. */
    "data_needs_ajustment.c_none",         /**< Event data_needs_ajustment.c_none. */
    "message_no_adjustment.u_response",    /**< Event message_no_adjustment.u_response. */
    "data_no_adjustment.c_none",           /**< Event data_no_adjustment.c_none. */
    "message_ball_front_check.u_response", /**< Event message_ball_front_check.u_response. */
    "data_ball_front_check.c_none",        /**< Event data_ball_front_check.c_none. */
    "message_goal_correction.u_response",  /**< Event message_goal_correction.u_response. */
    "data_goal_correction.c_none",         /**< Event data_goal_correction.c_none. */
    "message_no_goal.u_response",          /**< Event message_no_goal.u_response. */
    "data_no_goal.c_none",                 /**< Event data_no_goal.c_none. */
    "message_stop.u_response",             /**< Event message_stop.u_response. */
    "data_stop.c_none",                    /**< Event data_stop.c_none. */
    "message_continue.u_response",         /**< Event message_continue.u_response. */
    "data_continue.c_none",                /**< Event data_continue.c_none. */
    "message_move.c_trigger",              /**< Event message_move.c_trigger. */
    "data_move.c_none",                    /**< Event data_move.c_none. */
    "data_move.c_pRQWLNTV29I63",           /**< Event data_move.c_pRQWLNTV29I63. */
    "data_move.c_p36BWWUSNJC3L",           /**< Event data_move.c_p36BWWUSNJC3L. */
    "data_move.c_pD5RBQXC0II53",           /**< Event data_move.c_pD5RBQXC0II53. */
    "data_move.c_p06V0RK2WWOPV",           /**< Event data_move.c_p06V0RK2WWOPV. */
    "message_halt.c_trigger",              /**< Event message_halt.c_trigger. */
    "data_halt.c_none",                    /**< Event data_halt.c_none. */
    "data_halt.c_p0Q71S5GMFHQU",           /**< Event data_halt.c_p0Q71S5GMFHQU. */
};

/** Enumeration names. */
const char *enum_names[] = {
    "adjusting",
    "awaiting",
    "ball_found",
    "ball_in_front",
    "data_p8FSQAWK0EEMZ",
    "data_pNSBW9JQ6OU9X",
    "data_pRT63KQSV3XHI",
    "data_pV1KX6UX45Q6O",
    "data_pV31RVEZ6ALCC",
    "free",
    "goal_found",
    "in_service",
    "no_ball",
    "no_goal",
    "none",
    "obstructed",
    "stopped",
};

/* Constants. */


/* Functions. */


/* Input variables. */
controllerEnum message_scan_i_response_; /**< Input variable "E message_scan.i_response". */

/* State variables. */
controllerEnum component_BallDetector_;       /**< Discrete variable "E component_BallDetector". */
controllerEnum component_EmergencyStop_;      /**< Discrete variable "E component_EmergencyStop". */
controllerEnum component_GoalDetector_;       /**< Discrete variable "E component_GoalDetector". */
controllerEnum component_Scanner_v_distance_; /**< Discrete variable "E component_Scanner.v_distance". */
controllerEnum data_halt_;                    /**< Discrete variable "E data_halt". */
controllerEnum data_move_;                    /**< Discrete variable "E data_move". */

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
 * Execute code for event "data_ball_correction.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent0(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_ball_correction_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_ball_correction_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_ball_front_check.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent1(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_ball_front_check_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_ball_front_check_c_none_, FALSE);
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
 * Execute code for event "data_goal_correction.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent3(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_goal_correction_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_goal_correction_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_halt.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent4(void) {
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
 * Execute code for event "data_halt.c_p0Q71S5GMFHQU".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent5(void) {
    BoolType guard = ((data_halt_) == (_controller_none)) || ((data_halt_) == (_controller_data_p8FSQAWK0EEMZ));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_halt_c_p0Q71S5GMFHQU_, TRUE);
    #endif

    if ((data_halt_) == (_controller_none)) {
        data_halt_ = _controller_data_p8FSQAWK0EEMZ;
    } else if ((data_halt_) == (_controller_data_p8FSQAWK0EEMZ)) {
        data_halt_ = _controller_data_p8FSQAWK0EEMZ;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_halt_c_p0Q71S5GMFHQU_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_move.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent6(void) {
    BoolType guard = (((((data_move_) == (_controller_none)) && ((component_BallDetector_ != _controller_ball_found))) && (((component_BallDetector_ != _controller_adjusting)) && (((component_BallDetector_ != _controller_ball_in_front)) && ((component_BallDetector_ != _controller_no_ball))))) || ((((data_move_) == (_controller_data_pV31RVEZ6ALCC)) && ((component_BallDetector_ != _controller_ball_found))) && (((component_BallDetector_ != _controller_adjusting)) && (((component_BallDetector_ != _controller_ball_in_front)) && ((component_BallDetector_ != _controller_no_ball)))))) || (((((data_move_) == (_controller_data_pRT63KQSV3XHI)) && ((component_BallDetector_ != _controller_ball_found))) && (((component_BallDetector_ != _controller_adjusting)) && (((component_BallDetector_ != _controller_ball_in_front)) && ((component_BallDetector_ != _controller_no_ball))))) || (((((data_move_) == (_controller_data_pNSBW9JQ6OU9X)) && ((component_BallDetector_ != _controller_ball_found))) && (((component_BallDetector_ != _controller_adjusting)) && (((component_BallDetector_ != _controller_ball_in_front)) && ((component_BallDetector_ != _controller_no_ball))))) || ((((data_move_) == (_controller_data_pV1KX6UX45Q6O)) && ((component_BallDetector_ != _controller_ball_found))) && (((component_BallDetector_ != _controller_adjusting)) && (((component_BallDetector_ != _controller_ball_in_front)) && ((component_BallDetector_ != _controller_no_ball)))))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_none_, TRUE);
    #endif

    if ((((data_move_) == (_controller_none)) && ((component_BallDetector_ != _controller_ball_found))) && (((component_BallDetector_ != _controller_adjusting)) && (((component_BallDetector_ != _controller_ball_in_front)) && ((component_BallDetector_ != _controller_no_ball))))) {
        data_move_ = _controller_none;
    } else if ((((data_move_) == (_controller_data_pV31RVEZ6ALCC)) && ((component_BallDetector_ != _controller_ball_found))) && (((component_BallDetector_ != _controller_adjusting)) && (((component_BallDetector_ != _controller_ball_in_front)) && ((component_BallDetector_ != _controller_no_ball))))) {
        data_move_ = _controller_none;
    } else if ((((data_move_) == (_controller_data_pRT63KQSV3XHI)) && ((component_BallDetector_ != _controller_ball_found))) && (((component_BallDetector_ != _controller_adjusting)) && (((component_BallDetector_ != _controller_ball_in_front)) && ((component_BallDetector_ != _controller_no_ball))))) {
        data_move_ = _controller_none;
    } else if ((((data_move_) == (_controller_data_pNSBW9JQ6OU9X)) && ((component_BallDetector_ != _controller_ball_found))) && (((component_BallDetector_ != _controller_adjusting)) && (((component_BallDetector_ != _controller_ball_in_front)) && ((component_BallDetector_ != _controller_no_ball))))) {
        data_move_ = _controller_none;
    } else if ((((data_move_) == (_controller_data_pV1KX6UX45Q6O)) && ((component_BallDetector_ != _controller_ball_found))) && (((component_BallDetector_ != _controller_adjusting)) && (((component_BallDetector_ != _controller_ball_in_front)) && ((component_BallDetector_ != _controller_no_ball))))) {
        data_move_ = _controller_none;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_move.c_p06V0RK2WWOPV".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent7(void) {
    BoolType guard = ((((data_move_) == (_controller_none)) && ((component_BallDetector_) == (_controller_no_ball))) || (((data_move_) == (_controller_data_pV31RVEZ6ALCC)) && ((component_BallDetector_) == (_controller_no_ball)))) || ((((data_move_) == (_controller_data_pRT63KQSV3XHI)) && ((component_BallDetector_) == (_controller_no_ball))) || ((((data_move_) == (_controller_data_pNSBW9JQ6OU9X)) && ((component_BallDetector_) == (_controller_no_ball))) || (((data_move_) == (_controller_data_pV1KX6UX45Q6O)) && ((component_BallDetector_) == (_controller_no_ball)))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_p06V0RK2WWOPV_, TRUE);
    #endif

    if (((data_move_) == (_controller_none)) && ((component_BallDetector_) == (_controller_no_ball))) {
        data_move_ = _controller_data_pV1KX6UX45Q6O;
    } else if (((data_move_) == (_controller_data_pV31RVEZ6ALCC)) && ((component_BallDetector_) == (_controller_no_ball))) {
        data_move_ = _controller_data_pV1KX6UX45Q6O;
    } else if (((data_move_) == (_controller_data_pRT63KQSV3XHI)) && ((component_BallDetector_) == (_controller_no_ball))) {
        data_move_ = _controller_data_pV1KX6UX45Q6O;
    } else if (((data_move_) == (_controller_data_pNSBW9JQ6OU9X)) && ((component_BallDetector_) == (_controller_no_ball))) {
        data_move_ = _controller_data_pV1KX6UX45Q6O;
    } else if (((data_move_) == (_controller_data_pV1KX6UX45Q6O)) && ((component_BallDetector_) == (_controller_no_ball))) {
        data_move_ = _controller_data_pV1KX6UX45Q6O;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_p06V0RK2WWOPV_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_move.c_p36BWWUSNJC3L".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent8(void) {
    BoolType guard = ((((data_move_) == (_controller_none)) && ((component_BallDetector_) == (_controller_adjusting))) || (((data_move_) == (_controller_data_pV31RVEZ6ALCC)) && ((component_BallDetector_) == (_controller_adjusting)))) || ((((data_move_) == (_controller_data_pRT63KQSV3XHI)) && ((component_BallDetector_) == (_controller_adjusting))) || ((((data_move_) == (_controller_data_pNSBW9JQ6OU9X)) && ((component_BallDetector_) == (_controller_adjusting))) || (((data_move_) == (_controller_data_pV1KX6UX45Q6O)) && ((component_BallDetector_) == (_controller_adjusting)))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_p36BWWUSNJC3L_, TRUE);
    #endif

    if (((data_move_) == (_controller_none)) && ((component_BallDetector_) == (_controller_adjusting))) {
        data_move_ = _controller_data_pRT63KQSV3XHI;
    } else if (((data_move_) == (_controller_data_pV31RVEZ6ALCC)) && ((component_BallDetector_) == (_controller_adjusting))) {
        data_move_ = _controller_data_pRT63KQSV3XHI;
    } else if (((data_move_) == (_controller_data_pRT63KQSV3XHI)) && ((component_BallDetector_) == (_controller_adjusting))) {
        data_move_ = _controller_data_pRT63KQSV3XHI;
    } else if (((data_move_) == (_controller_data_pNSBW9JQ6OU9X)) && ((component_BallDetector_) == (_controller_adjusting))) {
        data_move_ = _controller_data_pRT63KQSV3XHI;
    } else if (((data_move_) == (_controller_data_pV1KX6UX45Q6O)) && ((component_BallDetector_) == (_controller_adjusting))) {
        data_move_ = _controller_data_pRT63KQSV3XHI;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_p36BWWUSNJC3L_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_move.c_pD5RBQXC0II53".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent9(void) {
    BoolType guard = ((((data_move_) == (_controller_none)) && ((component_BallDetector_) == (_controller_ball_in_front))) || (((data_move_) == (_controller_data_pV31RVEZ6ALCC)) && ((component_BallDetector_) == (_controller_ball_in_front)))) || ((((data_move_) == (_controller_data_pRT63KQSV3XHI)) && ((component_BallDetector_) == (_controller_ball_in_front))) || ((((data_move_) == (_controller_data_pNSBW9JQ6OU9X)) && ((component_BallDetector_) == (_controller_ball_in_front))) || (((data_move_) == (_controller_data_pV1KX6UX45Q6O)) && ((component_BallDetector_) == (_controller_ball_in_front)))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_pD5RBQXC0II53_, TRUE);
    #endif

    if (((data_move_) == (_controller_none)) && ((component_BallDetector_) == (_controller_ball_in_front))) {
        data_move_ = _controller_data_pNSBW9JQ6OU9X;
    } else if (((data_move_) == (_controller_data_pV31RVEZ6ALCC)) && ((component_BallDetector_) == (_controller_ball_in_front))) {
        data_move_ = _controller_data_pNSBW9JQ6OU9X;
    } else if (((data_move_) == (_controller_data_pRT63KQSV3XHI)) && ((component_BallDetector_) == (_controller_ball_in_front))) {
        data_move_ = _controller_data_pNSBW9JQ6OU9X;
    } else if (((data_move_) == (_controller_data_pNSBW9JQ6OU9X)) && ((component_BallDetector_) == (_controller_ball_in_front))) {
        data_move_ = _controller_data_pNSBW9JQ6OU9X;
    } else if (((data_move_) == (_controller_data_pV1KX6UX45Q6O)) && ((component_BallDetector_) == (_controller_ball_in_front))) {
        data_move_ = _controller_data_pNSBW9JQ6OU9X;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_pD5RBQXC0II53_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_move.c_pRQWLNTV29I63".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent10(void) {
    BoolType guard = ((((data_move_) == (_controller_none)) && ((component_BallDetector_) == (_controller_ball_found))) || (((data_move_) == (_controller_data_pV31RVEZ6ALCC)) && ((component_BallDetector_) == (_controller_ball_found)))) || ((((data_move_) == (_controller_data_pRT63KQSV3XHI)) && ((component_BallDetector_) == (_controller_ball_found))) || ((((data_move_) == (_controller_data_pNSBW9JQ6OU9X)) && ((component_BallDetector_) == (_controller_ball_found))) || (((data_move_) == (_controller_data_pV1KX6UX45Q6O)) && ((component_BallDetector_) == (_controller_ball_found)))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_pRQWLNTV29I63_, TRUE);
    #endif

    if (((data_move_) == (_controller_none)) && ((component_BallDetector_) == (_controller_ball_found))) {
        data_move_ = _controller_data_pV31RVEZ6ALCC;
    } else if (((data_move_) == (_controller_data_pV31RVEZ6ALCC)) && ((component_BallDetector_) == (_controller_ball_found))) {
        data_move_ = _controller_data_pV31RVEZ6ALCC;
    } else if (((data_move_) == (_controller_data_pRT63KQSV3XHI)) && ((component_BallDetector_) == (_controller_ball_found))) {
        data_move_ = _controller_data_pV31RVEZ6ALCC;
    } else if (((data_move_) == (_controller_data_pNSBW9JQ6OU9X)) && ((component_BallDetector_) == (_controller_ball_found))) {
        data_move_ = _controller_data_pV31RVEZ6ALCC;
    } else if (((data_move_) == (_controller_data_pV1KX6UX45Q6O)) && ((component_BallDetector_) == (_controller_ball_found))) {
        data_move_ = _controller_data_pV31RVEZ6ALCC;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_pRQWLNTV29I63_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_needs_ajustment.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent11(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_needs_ajustment_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_needs_ajustment_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_no_adjustment.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent12(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_no_adjustment_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_no_adjustment_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_no_ball.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent13(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_no_ball_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_no_ball_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_no_goal.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent14(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_no_goal_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_no_goal_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_scan.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent15(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_scan_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_scan_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_stop.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent16(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_stop_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_stop_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_ball_correction.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent17(void) {
    BoolType guard = ((((component_BallDetector_) == (_controller_awaiting)) || ((component_BallDetector_) == (_controller_ball_found))) || (((component_BallDetector_) == (_controller_no_ball)) || (((component_BallDetector_) == (_controller_ball_in_front)) || ((component_BallDetector_) == (_controller_adjusting))))) && ((((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_GoalDetector_) == (_controller_awaiting)) || (((component_GoalDetector_) == (_controller_goal_found)) || ((component_GoalDetector_) == (_controller_no_goal)))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_ball_correction_u_response_, TRUE);
    #endif

    if ((component_BallDetector_) == (_controller_awaiting)) {
        component_BallDetector_ = _controller_ball_found;
    } else if ((component_BallDetector_) == (_controller_ball_found)) {
        component_BallDetector_ = _controller_ball_found;
    } else if ((component_BallDetector_) == (_controller_no_ball)) {
        component_BallDetector_ = _controller_ball_found;
    } else if ((component_BallDetector_) == (_controller_ball_in_front)) {
        component_BallDetector_ = _controller_ball_in_front;
    } else if ((component_BallDetector_) == (_controller_adjusting)) {
        component_BallDetector_ = _controller_adjusting;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(message_ball_correction_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_ball_front_check.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent18(void) {
    BoolType guard = ((((component_BallDetector_) == (_controller_awaiting)) || ((component_BallDetector_) == (_controller_ball_found))) || (((component_BallDetector_) == (_controller_no_ball)) || (((component_BallDetector_) == (_controller_ball_in_front)) || ((component_BallDetector_) == (_controller_adjusting))))) && ((((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_GoalDetector_) == (_controller_awaiting)) || (((component_GoalDetector_) == (_controller_goal_found)) || ((component_GoalDetector_) == (_controller_no_goal)))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_ball_front_check_u_response_, TRUE);
    #endif

    if ((component_BallDetector_) == (_controller_awaiting)) {
        component_BallDetector_ = _controller_awaiting;
    } else if ((component_BallDetector_) == (_controller_ball_found)) {
        component_BallDetector_ = _controller_ball_in_front;
    } else if ((component_BallDetector_) == (_controller_no_ball)) {
        component_BallDetector_ = _controller_no_ball;
    } else if ((component_BallDetector_) == (_controller_ball_in_front)) {
        component_BallDetector_ = _controller_ball_in_front;
    } else if ((component_BallDetector_) == (_controller_adjusting)) {
        component_BallDetector_ = _controller_adjusting;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(message_ball_front_check_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_continue.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent19(void) {
    BoolType guard = ((((component_BallDetector_) == (_controller_awaiting)) || ((component_BallDetector_) == (_controller_ball_found))) || (((component_BallDetector_) == (_controller_no_ball)) || (((component_BallDetector_) == (_controller_ball_in_front)) || ((component_BallDetector_) == (_controller_adjusting))))) && ((((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_GoalDetector_) == (_controller_awaiting)) || (((component_GoalDetector_) == (_controller_goal_found)) || ((component_GoalDetector_) == (_controller_no_goal)))));
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
 * Execute code for event "message_goal_correction.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent20(void) {
    BoolType guard = ((((component_BallDetector_) == (_controller_awaiting)) || ((component_BallDetector_) == (_controller_ball_found))) || (((component_BallDetector_) == (_controller_no_ball)) || (((component_BallDetector_) == (_controller_ball_in_front)) || ((component_BallDetector_) == (_controller_adjusting))))) && ((((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_GoalDetector_) == (_controller_awaiting)) || (((component_GoalDetector_) == (_controller_goal_found)) || ((component_GoalDetector_) == (_controller_no_goal)))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_goal_correction_u_response_, TRUE);
    #endif

    if ((component_GoalDetector_) == (_controller_awaiting)) {
        component_GoalDetector_ = _controller_goal_found;
    } else if ((component_GoalDetector_) == (_controller_goal_found)) {
        component_GoalDetector_ = _controller_goal_found;
    } else if ((component_GoalDetector_) == (_controller_no_goal)) {
        component_GoalDetector_ = _controller_goal_found;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(message_goal_correction_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_halt.c_trigger".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent21(void) {
    BoolType guard = ((component_EmergencyStop_) == (_controller_stopped)) || ((component_Scanner_v_distance_) == (_controller_obstructed));
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
static BoolType execEvent22(void) {
    BoolType guard = ((component_EmergencyStop_) == (_controller_in_service)) && ((component_Scanner_v_distance_) == (_controller_free));
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
 * Execute code for event "message_needs_ajustment.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent23(void) {
    BoolType guard = ((((component_BallDetector_) == (_controller_awaiting)) || ((component_BallDetector_) == (_controller_ball_found))) || (((component_BallDetector_) == (_controller_no_ball)) || (((component_BallDetector_) == (_controller_ball_in_front)) || ((component_BallDetector_) == (_controller_adjusting))))) && ((((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_GoalDetector_) == (_controller_awaiting)) || (((component_GoalDetector_) == (_controller_goal_found)) || ((component_GoalDetector_) == (_controller_no_goal)))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_needs_ajustment_u_response_, TRUE);
    #endif

    if ((component_BallDetector_) == (_controller_awaiting)) {
        component_BallDetector_ = _controller_awaiting;
    } else if ((component_BallDetector_) == (_controller_ball_found)) {
        component_BallDetector_ = _controller_ball_found;
    } else if ((component_BallDetector_) == (_controller_no_ball)) {
        component_BallDetector_ = _controller_no_ball;
    } else if ((component_BallDetector_) == (_controller_ball_in_front)) {
        component_BallDetector_ = _controller_adjusting;
    } else if ((component_BallDetector_) == (_controller_adjusting)) {
        component_BallDetector_ = _controller_adjusting;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(message_needs_ajustment_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_no_adjustment.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent24(void) {
    BoolType guard = ((((component_BallDetector_) == (_controller_awaiting)) || ((component_BallDetector_) == (_controller_ball_found))) || (((component_BallDetector_) == (_controller_no_ball)) || (((component_BallDetector_) == (_controller_ball_in_front)) || ((component_BallDetector_) == (_controller_adjusting))))) && ((((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_GoalDetector_) == (_controller_awaiting)) || (((component_GoalDetector_) == (_controller_goal_found)) || ((component_GoalDetector_) == (_controller_no_goal)))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_no_adjustment_u_response_, TRUE);
    #endif

    if ((component_BallDetector_) == (_controller_awaiting)) {
        component_BallDetector_ = _controller_awaiting;
    } else if ((component_BallDetector_) == (_controller_ball_found)) {
        component_BallDetector_ = _controller_ball_found;
    } else if ((component_BallDetector_) == (_controller_no_ball)) {
        component_BallDetector_ = _controller_no_ball;
    } else if ((component_BallDetector_) == (_controller_ball_in_front)) {
        component_BallDetector_ = _controller_ball_in_front;
    } else if ((component_BallDetector_) == (_controller_adjusting)) {
        component_BallDetector_ = _controller_ball_in_front;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(message_no_adjustment_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_no_ball.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent25(void) {
    BoolType guard = ((((component_BallDetector_) == (_controller_awaiting)) || ((component_BallDetector_) == (_controller_ball_found))) || (((component_BallDetector_) == (_controller_no_ball)) || (((component_BallDetector_) == (_controller_ball_in_front)) || ((component_BallDetector_) == (_controller_adjusting))))) && ((((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_GoalDetector_) == (_controller_awaiting)) || (((component_GoalDetector_) == (_controller_goal_found)) || ((component_GoalDetector_) == (_controller_no_goal)))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_no_ball_u_response_, TRUE);
    #endif

    if ((component_BallDetector_) == (_controller_awaiting)) {
        component_BallDetector_ = _controller_no_ball;
    } else if ((component_BallDetector_) == (_controller_ball_found)) {
        component_BallDetector_ = _controller_no_ball;
    } else if ((component_BallDetector_) == (_controller_no_ball)) {
        component_BallDetector_ = _controller_no_ball;
    } else if ((component_BallDetector_) == (_controller_ball_in_front)) {
        component_BallDetector_ = _controller_ball_in_front;
    } else if ((component_BallDetector_) == (_controller_adjusting)) {
        component_BallDetector_ = _controller_adjusting;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(message_no_ball_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_no_goal.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent26(void) {
    BoolType guard = ((((component_BallDetector_) == (_controller_awaiting)) || ((component_BallDetector_) == (_controller_ball_found))) || (((component_BallDetector_) == (_controller_no_ball)) || (((component_BallDetector_) == (_controller_ball_in_front)) || ((component_BallDetector_) == (_controller_adjusting))))) && ((((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_GoalDetector_) == (_controller_awaiting)) || (((component_GoalDetector_) == (_controller_goal_found)) || ((component_GoalDetector_) == (_controller_no_goal)))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_no_goal_u_response_, TRUE);
    #endif

    if ((component_GoalDetector_) == (_controller_awaiting)) {
        component_GoalDetector_ = _controller_no_goal;
    } else if ((component_GoalDetector_) == (_controller_goal_found)) {
        component_GoalDetector_ = _controller_no_goal;
    } else if ((component_GoalDetector_) == (_controller_no_goal)) {
        component_GoalDetector_ = _controller_no_goal;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(message_no_goal_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_scan.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent27(void) {
    BoolType guard = ((((component_BallDetector_) == (_controller_awaiting)) || ((component_BallDetector_) == (_controller_ball_found))) || (((component_BallDetector_) == (_controller_no_ball)) || (((component_BallDetector_) == (_controller_ball_in_front)) || ((component_BallDetector_) == (_controller_adjusting))))) && ((((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_GoalDetector_) == (_controller_awaiting)) || (((component_GoalDetector_) == (_controller_goal_found)) || ((component_GoalDetector_) == (_controller_no_goal)))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_scan_u_response_, TRUE);
    #endif

    component_Scanner_v_distance_ = message_scan_i_response_;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_scan_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_stop.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent28(void) {
    BoolType guard = ((((component_BallDetector_) == (_controller_awaiting)) || ((component_BallDetector_) == (_controller_ball_found))) || (((component_BallDetector_) == (_controller_no_ball)) || (((component_BallDetector_) == (_controller_ball_in_front)) || ((component_BallDetector_) == (_controller_adjusting))))) && ((((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_GoalDetector_) == (_controller_awaiting)) || (((component_GoalDetector_) == (_controller_goal_found)) || ((component_GoalDetector_) == (_controller_no_goal)))));
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

        if (execEvent0()) continue;  /* (Try to) perform event "data_ball_correction.c_none". */
        if (execEvent1()) continue;  /* (Try to) perform event "data_ball_front_check.c_none". */
        if (execEvent2()) continue;  /* (Try to) perform event "data_continue.c_none". */
        if (execEvent3()) continue;  /* (Try to) perform event "data_goal_correction.c_none". */
        if (execEvent4()) continue;  /* (Try to) perform event "data_halt.c_none". */
        if (execEvent5()) continue;  /* (Try to) perform event "data_halt.c_p0Q71S5GMFHQU". */
        if (execEvent6()) continue;  /* (Try to) perform event "data_move.c_none". */
        if (execEvent7()) continue;  /* (Try to) perform event "data_move.c_p06V0RK2WWOPV". */
        if (execEvent8()) continue;  /* (Try to) perform event "data_move.c_p36BWWUSNJC3L". */
        if (execEvent9()) continue;  /* (Try to) perform event "data_move.c_pD5RBQXC0II53". */
        if (execEvent10()) continue;  /* (Try to) perform event "data_move.c_pRQWLNTV29I63". */
        if (execEvent11()) continue;  /* (Try to) perform event "data_needs_ajustment.c_none". */
        if (execEvent12()) continue;  /* (Try to) perform event "data_no_adjustment.c_none". */
        if (execEvent13()) continue;  /* (Try to) perform event "data_no_ball.c_none". */
        if (execEvent14()) continue;  /* (Try to) perform event "data_no_goal.c_none". */
        if (execEvent15()) continue;  /* (Try to) perform event "data_scan.c_none". */
        if (execEvent16()) continue;  /* (Try to) perform event "data_stop.c_none". */
        if (execEvent21()) continue;  /* (Try to) perform event "message_halt.c_trigger". */
        if (execEvent22()) continue;  /* (Try to) perform event "message_move.c_trigger". */
        break; /* No event fired, done with discrete steps. */
    }
}

/** First model call, initializing, and performing discrete events before the first time step. */
void controller_EngineFirstStep(void) {
    InitConstants();

    model_time = 0.0;
    controller_AssignInputVariables();
    component_BallDetector_ = _controller_awaiting;
    component_EmergencyStop_ = _controller_in_service;
    component_GoalDetector_ = _controller_awaiting;
    component_Scanner_v_distance_ = _controller_obstructed;
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
        case data_ball_correction_c_none_:
            return execEvent0();
        case data_ball_front_check_c_none_:
            return execEvent1();
        case data_continue_c_none_:
            return execEvent2();
        case data_goal_correction_c_none_:
            return execEvent3();
        case data_halt_c_none_:
            return execEvent4();
        case data_halt_c_p0Q71S5GMFHQU_:
            return execEvent5();
        case data_move_c_none_:
            return execEvent6();
        case data_move_c_p06V0RK2WWOPV_:
            return execEvent7();
        case data_move_c_p36BWWUSNJC3L_:
            return execEvent8();
        case data_move_c_pD5RBQXC0II53_:
            return execEvent9();
        case data_move_c_pRQWLNTV29I63_:
            return execEvent10();
        case data_needs_ajustment_c_none_:
            return execEvent11();
        case data_no_adjustment_c_none_:
            return execEvent12();
        case data_no_ball_c_none_:
            return execEvent13();
        case data_no_goal_c_none_:
            return execEvent14();
        case data_scan_c_none_:
            return execEvent15();
        case data_stop_c_none_:
            return execEvent16();
        case message_ball_correction_u_response_:
            return execEvent17();
        case message_ball_front_check_u_response_:
            return execEvent18();
        case message_continue_u_response_:
            return execEvent19();
        case message_goal_correction_u_response_:
            return execEvent20();
        case message_halt_c_trigger_:
            return execEvent21();
        case message_move_c_trigger_:
            return execEvent22();
        case message_needs_ajustment_u_response_:
            return execEvent23();
        case message_no_adjustment_u_response_:
            return execEvent24();
        case message_no_ball_u_response_:
            return execEvent25();
        case message_no_goal_u_response_:
            return execEvent26();
        case message_scan_u_response_:
            return execEvent27();
        case message_stop_u_response_:
            return execEvent28();
        default:
            return FALSE;
    }
}
