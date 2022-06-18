using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

public class HandPresence : MonoBehaviour {
    InputDevice rightController, leftController;
    void Start() {
        var inputDevices = new List<InputDevice>();
        InputDevices.GetDevices(inputDevices);

        foreach (var device in inputDevices) {
            Debug.Log(string.Format("Device found with name '{0}' and characteristics '{1}'", device.name, device.characteristics));
        }
    }

    // Update is called once per frame
    void Update() {
        rightController.TryGetFeatureValue(CommonUsages.primaryButton, out bool primaryButtonValue);
        if (primaryButtonValue) {
            Debug.Log("Primary button");
        }
        var inputDevices = new List<InputDevice>();
        InputDevices.GetDevices(inputDevices);

        foreach (var device in inputDevices) {
            Debug.Log(string.Format("Device found with name '{0}' and characteristics '{1}'", device.name, device.characteristics));
        }
    }
}
