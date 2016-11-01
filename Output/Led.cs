using System;
using Windows.Devices.Gpio;

namespace SuperEvilMasterminds.Iot.Devices.Output
{
    public sealed class Led : ISyncDevice
    {
        public bool Value
        {
            get
            {
                EnsureInitialised();
                return m_ledPin.Read() == GpioPinValue.High;
            }
            set
            {
                EnsureInitialised();
                m_ledPin.Write(value ? GpioPinValue.High : GpioPinValue.Low);
            }
        }

        public readonly int Pin;

        private GpioPin m_ledPin;
        private bool m_initialised;

        public Led(int pin)
        {
            Pin = pin;
        }

        private void EnsureInitialised()
        {
            if (m_initialised)
                return;

            throw new InvalidOperationException("This device is not initialised.");
        }

        public bool Initialise()
        {
            if (m_initialised)
                return true;

            var gpio = GpioController.GetDefault();
            if (gpio == null)
                return false;

            GpioOpenStatus status;
            if (!gpio.TryOpenPin(Pin, GpioSharingMode.Exclusive, out m_ledPin, out status))
                return false;

            m_initialised = true;

            try
            {
                m_ledPin.SetDriveMode(GpioPinDriveMode.Output);
                Value = false;
            }
            catch (Exception)
            {
                m_initialised = false;
            }

            return m_initialised;
        }

        public bool Toggle()
        {
            var newValue = !Value;
            Value = newValue;
            return newValue;
        }
    }
}
