#ifndef STEPPERAXIS_HH
#define STEPPERAXIS_HH

#include "Configuration.hh"

#if STEPPER_COUNT > 0
#include "StepperInterface.hh"

/// The stepper axis module implements a driver for a single stepper axis. It is designed
/// to be accessed via the Steppers namespace, and uses a StepperInterface to talk to the
/// actual hardware.
/// \ingroup SoftwareLibraries
class StepperAxis
{
protected:
        const StepperInterface* interface;    ///< Interface this axis is connected to

public:
        bool direction;                 ///< True for positive, false for negative
        int32_t delta;                  ///< Amount to increment counter per tick
        volatile int32_t position;      ///< Current position of this axis, in steps
        volatile int32_t counter;       ///< Step counter; represents the proportion of
                                        ///< a step so far passed.  When the counter hits
                                        ///< zero, a step is taken.

protected:
#if defined(SINGLE_SWITCH_ENDSTOPS) && (SINGLE_SWITCH_ENDSTOPS == 1)
        volatile bool prev_direction;   ///< Record the previous direction for endstop detection
        volatile int32_t endstop_play;  ///< Amount to move while endstop triggered, to see which way to move
        
        enum endstop_status_t {         ///< State of the endstop
            ESS_UNKNOWN,
            ESS_TRAVELING,
            ESS_AT_MAXIMUM,
            ESS_AT_MINIMUM
        };
        
        volatile endstop_status_t endstop_status;

        // If we started with an endstop triggered, then we don't know where 
        // we are. We can go this many steps either way until we find out.
        const static uint16_t ENDSTOP_DEFAULT_PLAY =10000;
        const static uint16_t ENDSTOP_DEBOUNCE =20;

#endif //SINGLE_SWITCH_ENDSTOPS
        // Return true if the endstop for the current direction is triggered.
        inline bool checkEndstop(const bool isHoming);

public:
        /// Construct a stepper axis with a null interface
        StepperAxis();

        /// Construct a stepper axis, using the given stepper
        /// interface
        /// \param[in] Stepper interface to use
        StepperAxis(const StepperInterface* stepper_interface);

        /// Set the target position for the axis to travel to.
        /// \param[in] target_in Postion to move to, in steps
        /// \param[in] relative If true, consider the target position
        ///                     to be relative to the current position.
        // Returns the amount of movement required
        int32_t setTarget(const int32_t target_in, bool relative);

        /// Start a homing procedure
        /// \param[in] direction_in If true, home in the positive direction.
        void setHoming(const bool direction_in);

        /// Set whether the stepper motor driver on the given axis should be enabled
        /// \param[in] enable If true, enable the axis; otherwise, disable it.
        inline void enableStepper(bool enable) { interface->setEnabled(enable); }

        /// Reset to initial state
        void reset();

        /// Handle interrupt for the given axis.
        /// \param[in] intervals Intervals that have passed since the previous interrupt
        void doInterrupt(const int32_t intervals);

        /// Run the next step of the homing procedure.
        /// \param[in] intervals Intervals that have passed since the previous interrupt
        /// \return True if the axis is still homing.
        bool doHoming(const int32_t intervals);
};
#endif
#endif // STEPPERAXIS_HH
