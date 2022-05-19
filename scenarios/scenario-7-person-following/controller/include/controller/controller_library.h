/**
 * Headers and inline functions of the runtime support for the CIF to C99 translation.
 */

#ifndef CIF_C_CONTROLLER_CODEGEN_LIBRARY_H
#define CIF_C_CONTROLLER_CODEGEN_LIBRARY_H

/**
 * Support library for the CIF to C code generation.
 *
 * - CIF 'bool' maps to _Bool (BoolType typedef). There are also TRUE and FALSE
 *   constants.
 * - CIF 'int' maps to int32_t (IntType typedef). MinInt and MaxInt limits
 *   denote the smallest and biggest value that can be used for the integer type.
 * - CIF 'real' maps to double, wihout NaN, +Inf, -Inf, and -0.0 (RealType
 *   typedef).
 * - CIF 'string' maps to the 'StringType', with a compile-time MAX_STRING_SIZE
 *   limit for the maximal length strings that can be handled. The limit
 *   includes the terminating '\0' character.
 *
 * - The library does not attempt to provide nice error messages, since a crash
 *   is inevitable, and the crash location gives sufficient information what is
 *   wrong. 'assert' is used overflow/underflow/zero-division detection.
 */

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <assert.h>

/* Define max string length if not defined. */
#ifndef MAX_STRING_SIZE
#define MAX_STRING_SIZE 128
#endif

typedef _Bool BoolType;
typedef int32_t IntType;
typedef double RealType;

static const BoolType TRUE = 1;
static const BoolType FALSE = 0;
static const IntType MaxInt = INT32_MAX;
static const IntType MinInt = INT32_MIN;

/** String data type. */
struct StringTypeStruct {
    char data[MAX_STRING_SIZE];
};
typedef struct StringTypeStruct StringType;

/* String type support runtime code. */
void StringTypeCopyText(StringType *s, const char *text);
void StringTypeConcat(StringType *dest, StringType *left, StringType *right);

/**
 * Get the length of a string.
 * @param str String to inspect.
 * @return Length of the given string.
 */
static inline IntType StringTypeSize(StringType *str) {
    /* strnlen */
    IntType length = 0;
    const char *k = str->data;
    while (length < MAX_STRING_SIZE && *k) { length++; k++; }
    assert(length < MAX_STRING_SIZE); // String should have a terminating nul-character.

    return length;
}

BoolType StringTypeEquals(StringType *left, StringType *right);
void StringTypeProject(StringType *dst, StringType *src, IntType index);

/* Cast functions. */
BoolType StringToBool(StringType *s);
IntType  StringToInt(StringType *s);
RealType StringToReal(StringType *s);

void BoolToString(BoolType b, StringType *s);
void  IntToString(IntType i, StringType *s);
void RealToString(RealType r, StringType *s);

/* Format functions. */
enum FormatFlags {
    FMTFLAGS_NONE   = 0,
    FMTFLAGS_LEFT   = 1 << 0,
    FMTFLAGS_SIGN   = 1 << 1,
    FMTFLAGS_SPACE  = 1 << 2,
    FMTFLAGS_ZEROES = 1 << 3,
    FMTFLAGS_GROUPS = 1 << 4,
};

int StringTypeAppendText(StringType *s, int end, int flags, int width, const char *text);
int BoolTypePrint(BoolType b, char *dest, int start, int end);
int IntTypePrint(IntType i, char *dest, int start, int end);
int RealTypePrint(RealType r, char *dest, int start, int end);
int StringTypePrintRaw(StringType *s, char *dest, int start, int end);
int StringTypePrintEscaped(StringType *s, char *dest, int start, int end);

/**
 * Get absolute value of an integer.
 * @param value Value to convert.
 * @return The absolute value of \a value.
 */
static inline IntType IntegerAbs(IntType value) {
    assert(value != MinInt);
    return (value < 0) ? -value : value;
}

/**
 * Integer division, while checking for overflow and zero division.
 * @param a Numerator of the division.
 * @param b Denominator of the division.
 * @return Integer value of the division closest to \c 0.
 */
static inline IntType IntegerDiv(IntType a, IntType b) {
    assert(a != MinInt || b != -1);
    assert(b != 0);
    return a / b;
}

/**
 * Integer remainder, while checking for zero division.
 * @param a Numerator of the division.
 * @param b Denominator of the division.
 * @return Difference between \a a and the integer division.
 * @note The invariant is a == b * IntegerDiv(a, b) + IntegerMod(a, b)
 */
// Integer mod(a, b) with zero divide detection.
static inline IntType IntegerMod(IntType a, IntType b) {
    assert(b != 0);
    return a % b;
}

/**
 * Get absolute value of a real number.
 * @param value Value to convert.
 * @return The absolute value of \a value.
 */
static inline RealType RealAbs(RealType value) {
    value = (value < 0) ? -value : value;
    assert(isfinite(value));
    return (value == -0.0) ? 0.0 : value;
}

/**
 * Add two integers, checking for overflow or underflow.
 * @param a First value to add.
 * @param b Second value to add.
 * @return The sum of both values, iff there is no overflow or underflow.
 */
static inline IntType IntegerAdd(IntType a, IntType b) {
    int64_t sum = a;
    sum += b;

    assert(sum >= MinInt && sum <= MaxInt);
    return sum;
}

/**
 * Add two real numbers, while checking for overflow and underflow, and normalizing the result.
 * @param a First value to add.
 * @param b Second value to add.
 * @return The normalized sum.
 */
static inline RealType RealAdd(RealType a, RealType b) {
    a = a + b;
    assert(isfinite(a));
    return (a == -0.0) ? 0.0 : a;
}

/**
 * Subtract two integer values, while checking for underflow or overflow.
 * @param a First value to subtract.
 * @param b Second value to subtract.
 * @return The subtraction result.
 */
static inline IntType IntegerSubtract(IntType a, IntType b) {
    int64_t result = a;
    result -= b;

    assert(result >= MinInt && result <= MaxInt);
    return result;
}

/**
 * Subtract two real numbers, while checking for underflow or overflow, and normalizing the result.
 * @param a First value to subtract.
 * @param b Second value to subtract.
 * @return The normalized subtraction result.
 */
static inline RealType RealSubtract(RealType a, RealType b) {
    a = a - b;
    assert(isfinite(a));
    return (a == -0.0) ? 0.0 : a;
}

/**
 * Divide two real number, while checking for underflow or overflow, and normalizing the result.
 * @param a Numerator value of the division.
 * @param b Denominator value to division.
 * @return The normalize division result.
 */
static inline RealType RealDivision(RealType a, RealType b) {
    assert(b != 0.0);
    a = a / b;
    assert(isfinite(a));
    return (a == -0.0) ? 0.0 : a;
}

/**
 * Retrieve the biggest value of two integers.
 * @param a First integer to inspect.
 * @param b Second integer to inspect.
 * @return the biggest value of \a a and \a b.
 */
static inline IntType IntegerMax(IntType a, IntType b) {
    return (a > b) ? a : b;
}

/**
 * Retrieve the biggest value of two real numbers.
 * @param a First integer to inspect.
 * @param b Second integer to inspect.
 * @return the biggest value of \a a and \a b.
 */
static inline RealType RealMax(RealType a, RealType b) {
    return (a > b) ? a : b;
}

/**
 * Retrieve the smallest value of two integers.
 * @param a First integer to inspect.
 * @param b Second integer to inspect.
 * @return the smallest value of \a a and \a b.
 */
static inline IntType IntegerMin(IntType a, IntType b) {
    return (a < b) ? a : b;
}

/**
 * Retrieve the smallest value of two real numbers.
 * @param a First integer to inspect.
 * @param b Second integer to inspect.
 * @return the smallest value of \a a and \a b.
 */
static inline RealType RealMin(RealType a, RealType b) {
    return (a < b) ? a : b;
}

/**
 * Multiply two integers while checking for overflow.
 * @param a First integer to multiply.
 * @param b Second integer to multiply.
 * @return The multiplication result.
 */
static inline IntType IntegerMultiply(IntType a, IntType b) {
    int64_t result = a;
    result *= b;

    assert(result >= MinInt && result <= MaxInt);
    return result;
}

/**
 * Multiply two real numbers.
 * @param a First real number to multiply.
 * @param b Second real number to multiply.
 * @return The multiplication result.
 */
static inline RealType RealMultiply(RealType a, RealType b) {
    a *= b;
    assert(isfinite(a));
    return (a == -0.0) ? 0.0 : a;
}

/**
 * Negate the given integer value while checking for overflow.
 * @param a Value to negate.
 * @return The negated \c -x value.
 */
static inline IntType IntegerNegate(IntType a) {
    assert(a != MinInt);
    return -a;
}

/**
 * Negate the given real number while normalizing the result.
 * @param a Value to negate.
 * @return The negated \c -x value.
 */
static inline RealType RealNegate(RealType a) {
    return (a == 0.0) ? a : -a;
}

/**
 * Compute the sign of an integer value.
 * @param value Value to convert.
 * @return Positive value if \a value is more than zero, a negative value if \a
 *      value is less than zero, else zero.
 */
static inline IntType IntegerSign(IntType value) {
    if (value > 0) return 1;
    if (value < 0) return -1;
    return 0;
}

/**
 * Compute the sign of a real number.
 * @param value Value to convert.
 * @return Positive value if \a value is more than zero, a negative value if \a
 *      value is less than zero, else zero.
 */
static inline IntType RealSign(RealType value) {
    if (value > 0) return 1;
    if (value < 0) return -1;
    return 0;
}

/**
 * Compute acos(x) while checking for underflow and overflow, and normalizing the result.
 * @param arg Argument of the function.
 * @return The function result.
 */
static inline RealType RealAcos(RealType arg) {
    RealType result = acos(arg);
    assert(isfinite(result));
    return result == -0.0 ? 0.0 : result;
}

/**
 * Compute asin(x) while checking for underflow and overflow, and normalizing the result.
 * @param arg Argument of the function.
 * @return The function result.
 */
static inline RealType RealAsin(RealType arg) {
    RealType result = asin(arg);
    assert(isfinite(result));
    return result == -0.0 ? 0.0 : result;
}

/**
 * Compute atan(x) while checking for underflow and overflow, and normalizing the result.
 * @param arg Argument of the function.
 * @return The function result.
 */
static inline RealType RealAtan(RealType arg) {
    RealType result = atan(arg);
    assert(isfinite(result));
    return result == -0.0 ? 0.0 : result;
}

/**
 * Compute cos(x) while checking for underflow and overflow, and normalizing the result.
 * @param arg Argument of the function.
 * @return The function result.
 */
static inline RealType RealCos(RealType arg) {
    RealType result = cos(arg);
    assert(isfinite(result));
    return result == -0.0 ? 0.0 : result;
}

/**
 * Compute sin(x) while checking for underflow and overflow, and normalizing the result.
 * @param arg Argument of the function.
 * @return The function result.
 */
static inline RealType RealSin(RealType arg) {
    RealType result = sin(arg);
    assert(isfinite(result));
    return result == -0.0 ? 0.0 : result;
}

/**
 * Compute tan(x) while checking for underflow and overflow, and normalizing the result.
 * @param arg Argument of the function.
 * @return The function result.
 */
static inline RealType RealTan(RealType arg) {
    RealType result = tan(arg);
    assert(isfinite(result));
    return result == -0.0 ? 0.0 : result;
}

/**
 * Compute e**x while checking for underflow and overflow, and normalizing the result.
 * @param arg Argument of the function.
 * @return The function result.
 */
static inline RealType RealExp(RealType arg) {
    RealType result = exp(arg);
    assert(isfinite(result));
    return result == -0.0 ? 0.0 : result;
}

/**
 * Compute log_10(x) while checking for underflow and overflow, and normalizing the result.
 * @param arg Argument of the function.
 * @return The function result.
 */
static inline RealType RealLog(RealType arg) {
    RealType result = log10(arg);
    assert(isfinite(result));
    return result == -0.0 ? 0.0 : result;
}

/**
 * Compute ln(x) while checking for underflow and overflow, and normalizing the result.
 * @param arg Argument of the function.
 * @return The function result.
 */
static inline RealType RealLn(RealType arg) {
    RealType result = log(arg);
    assert(isfinite(result));
    return result == -0.0 ? 0.0 : result;
}

/**
 * Compute the square root of a real number while checking for underflow and overflow.
 * @param a Value to convert.
 * @return The square root of the given value \c x**(1/2).
 */
static inline RealType RealSqrt(RealType a) {
    a = sqrt(a);
    assert(isfinite(a));
    return a; // Cannot be -0.0.
}

/**
 * Compute the cube root of a real number while checking for underflow and overflow.
 * @param a Value to convert.
 * @return The cube root of the given value \c x**(1/3).
 */
static inline RealType RealCbrt(RealType a) {
    a = cbrt(a);
    assert(isfinite(a));
    return (a == -0.0) ? 0.0 : a;
}

/* Real to int conversions. */
IntType CeilFunction(RealType value);
IntType FloorFunction(RealType a);
IntType RoundFunction(RealType value);
IntType IntegerPower(IntType x, IntType y);

RealType ScaleFunction(RealType val, RealType inmin, RealType inmax,
                       RealType outmin, RealType outmax);

/* Standard library math functions. */
RealType RealAcos(RealType arg);
RealType RealAsin(RealType arg);
RealType RealAtan(RealType arg);

RealType RealCos(RealType arg);
RealType RealSin(RealType arg);
RealType RealTan(RealType arg);

RealType RealExp(RealType arg);
RealType RealLog(RealType arg);
RealType RealLn(RealType arg);

#endif

