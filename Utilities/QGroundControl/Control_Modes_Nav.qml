// This is an example Custom Command Qml file. You have full access to the entire Qml language
// for creating any user interface you like. From the ui you can affect the following changes
// with respect to your vehicle:
//    1) Sending COMMAND_LONG commands out over mavlink using QGCButton control
//    2) Modifying parameters
//
// When developing custom Qml file implementations. You must restart QGroundControl to pick up
// the changes. You need to do this even if you select Clear Qml file. Not sure what at the this
// point. Qt must be caching the files somewhere.

import QtQuick 2.2

import QGroundControl.Controls 1.0
import QGroundControl.FactSystem 1.0
import QGroundControl.FactControls 1.0

Item {

    // Your own custom changes start here - everything else above is always required

    Column {
        // The QGCButton control is provided by QGroundControl.Controls. It is a wrapper around
        // the standard Qml Button element which using the default QGC font and color palette.

        QGCButton {
            text: "Unarm"
            onClicked: controller.sendCommand(176, 50, 0, 64, 0, 0, 0, 0, 0, 0)
        }
        QGCButton {
            text: "MODE: Armed-Manual"
            onClicked: controller.sendCommand(176, 50, 0, 192, 0, 0, 0, 0, 0, 0)
        }
        QGCButton {
            text: "MODE: Armed-Stabalize"
            onClicked: controller.sendCommand(176, 50, 0, 208, 0, 0, 0, 0, 0, 0)
        }
        QGCButton {
            text: "MODE: Armed-Guided"
            onClicked: controller.sendCommand(176, 50, 0, 216, 0, 0, 0, 0, 0, 0)
        }
        QGCButton {
            text: "MODE: Armed-Auto"
            onClicked: controller.sendCommand(176, 50, 0, 156, 0, 0, 0, 0, 0, 0)
        }
        QGCButton {
            text: "NAV: Take-off"
            onClicked: controller.sendCommand(22, 50, 0, 1, 0, 0, 0, 0, 0, 0)
        }
        //QGCButton {
        //    text: "NAV: Return Home"
        //    onClicked: controller.sendCommand(20, 50, 0, 1, 0, 0, 0, 0, 0, 0)
        //}
        QGCButton {
            text: "NAV: Land"
            onClicked: controller.sendCommand(21, 50, 0, 0, 0, 0, 0, 0, 0, 0)
        }
        QGCButton {
            text: "NAV: Start"
            onClicked: controller.sendCommand(300, 50, 0, 0, 0, 0, 0, 0, 0, 0)
        }
        QGCButton {
            text: "NAV: Pause"
            onClicked: controller.sendCommand(252, 50, 0, 0, 2, 0, 0, 0, 0, 0)
        }
        QGCButton {
            text: "NAV: Continue/Skip"
            onClicked: controller.sendCommand(252, 50, 0, 1, 0, 0, 0, 0, 0, 0)
        }

        // The FactTextField control is provides by GroundControl.FactControls. It is a wrapper
        // around the Qml TextField element which allows you to bind it directly to any parameter.
        // The parameter is changed automatically when you click enter or click away from the field.
        // Understand that there is currently no value validation. So you may crash your vehicle by
        // setting a parameter to an incorrect value. Validation will come in the future.

        // Be very careful when referencing parameters. If you specify a parameter which does not exist
        // QGroundControl will warn and shutdown.

        //FactTextField {
        //    fact: Fact { name: "MAV_SYS_ID" }
        //}
    }

    // Your own custom changes end here - everything else below is always required
}