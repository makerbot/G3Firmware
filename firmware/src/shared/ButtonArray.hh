#ifndef BUTTONARRAY_HH
#define BUTTONARRAY_HH

#include <util/atomic.h>
#include "Types.hh"

// TODO: Make this an interface?

/// The button array modules manages an array of buttons, and maintains
/// a buffer of the last pressed button. It has two entry points: a fast
/// #scanButtons, which is a fast button scanning routine that should be
/// called from an interrupt, and #getButton, which should be called by a
/// slow loop that has time to respond to the button.
///
/// Porting Notes:
/// This modules uses low-level port registers, and must be re-written for
/// each board archetecture. This should be done by adding a ButtonArray.cc
/// definition in the board directory.
/// \ingroup HardwareLibraries
class ButtonArray {
private:
        uint8_t buttonPress;
        bool buttonPressWaiting;

public:
        /// Representation of the different buttons available on the keypad
        enum ButtonName {
                ZERO            = 1,
                ZMINUS          = 2,
                ZPLUS           = 3,
                YMINUS          = 4,
                YPLUS           = 5,
                XMINUS          = 6,
                XPLUS           = 7,
                CANCEL          = 11,
                OK              = 12
        };

        void init();

        // Returns true if any of the button states have changed.
        void scanButtons();

        bool getButton(ButtonName& button);
};


#endif // BUTTONARRAY_HH
