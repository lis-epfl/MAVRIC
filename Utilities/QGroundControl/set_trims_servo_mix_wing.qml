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
            text: "Save trims"
            onClicked: controller.sendCommand(183, 50, 0, 1, 0, 0, 0, 0, 0, 0)
        }
        QGCButton {
            text: "Reset trims"
            onClicked: controller.sendCommand(183, 50, 0, 0, 1, 0, 0, 0, 0, 0)
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