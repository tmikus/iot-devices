using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Windows.Devices.Enumeration;
using Windows.Devices.Gpio;
using Windows.Devices.I2c;

namespace SuperEvilMasterminds.Iot.Devices.Accelerometers
{
    public sealed class MpuSensorValue
    {
        public float AccelerationX { get; set; }
        public float AccelerationY { get; set; }
        public float AccelerationZ { get; set; }
        public float GyroX { get; set; }
        public float GyroY { get; set; }
        public float GyroZ { get; set; }
    }

    public sealed class MpuSensorEventArgs : EventArgs
    {
        public byte Status { get; set; }
        public float SamplePeriod { get; set; }
        public MpuSensorValue[] Values { get; set; }
    }

    public sealed class Mpu6050 : IDisposable
    {
        public event EventHandler<MpuSensorEventArgs> SensorInterruptEvent;

        private const byte Address = 0x68;
        private const byte PwrMgmt1 = 0x6B;
        private const byte SmplrtDiv = 0x19;
        private const byte Config = 0x1A;
        private const byte GyroConfig = 0x1B;
        private const byte AccelConfig = 0x1C;
        private const byte FifoEn = 0x23;
        private const byte IntEnable = 0x38;
        private const byte IntStatus = 0x3A;
        private const byte UserCtrl = 0x6A;
        private const byte FifoCount = 0x72;
        private const byte FifoRW = 0x74;
        private const int SensorBytes = 12;

        private readonly int m_interruptPinNumber;
        private I2cDevice m_mpu6050Device;
        private GpioController m_ioController;
        private GpioPin m_interruptPin;
        private bool m_disposed; // To detect redundant calls

        public Mpu6050(int interruptPinNumber = 18)
        {
            m_interruptPinNumber = interruptPinNumber;
        }

        ~Mpu6050()
        {
            // Do not change this code. Put cleanup code in Dispose(bool disposing) above.
            Dispose(false);
        }

        // This code added to correctly implement the disposable pattern.
        public void Dispose()
        {
            // Do not change this code. Put cleanup code in Dispose(bool disposing) above.
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        private void Dispose(bool disposing)
        {
            if (m_disposed)
                return;

            m_interruptPin.Dispose();

            if (m_mpu6050Device != null)
            {
                m_mpu6050Device.Dispose();
                m_mpu6050Device = null;
            }

            m_disposed = true;
        }

        private byte ReadByte(byte regAddr)
        {
            var buffer = new byte[1];
            buffer[0] = regAddr;
            var value = new byte[1];
            m_mpu6050Device.WriteRead(buffer, value);
            return value[0];
        }

        private byte[] ReadBytes(byte regAddr, int length)
        {
            var values = new byte[length];
            var buffer = new byte[1];
            buffer[0] = regAddr;
            m_mpu6050Device.WriteRead(buffer, values);
            return values;
        }

        private ushort ReadWord(byte address)
        {
            var buffer = ReadBytes(address, 2);
            return (ushort) ((buffer[0] << 8) | buffer[1]);
        }

        private void WriteByte(byte regAddr, byte data)
        {
            var buffer = new byte[2];
            buffer[0] = regAddr;
            buffer[1] = data;
            m_mpu6050Device.Write(buffer);
        }

        //private void WriteBytes(byte regAddr, byte[] values)
        //{
        //    var buffer = new byte[1 + values.Length];
        //    buffer[0] = regAddr;
        //    Array.Copy(values, 0, buffer, 1, values.Length);
        //    m_mpu6050Device.Write(buffer);
        //}

        public async Task InitHardware()
        {
            try
            {
                m_ioController = GpioController.GetDefault();
                m_interruptPin = m_ioController.OpenPin(m_interruptPinNumber);
                m_interruptPin.Write(GpioPinValue.Low);
                m_interruptPin.SetDriveMode(GpioPinDriveMode.Input);
                m_interruptPin.ValueChanged += Interrupt;

                var aqs = I2cDevice.GetDeviceSelector();
                var collection = await DeviceInformation.FindAllAsync(aqs);

                var settings = new I2cConnectionSettings(Address)
                {
                    BusSpeed = I2cBusSpeed.FastMode,
                    SharingMode = I2cSharingMode.Exclusive
                };
                m_mpu6050Device = await I2cDevice.FromIdAsync(collection[0].Id, settings);

                await Task.Delay(3); // wait power up sequence

                WriteByte(PwrMgmt1, 0x80); // reset the device
                await Task.Delay(100);
                WriteByte(PwrMgmt1, 0x2);
                WriteByte(UserCtrl, 0x04); //reset fifo

                WriteByte(PwrMgmt1, 1); // clock source = gyro x
                WriteByte(GyroConfig, 0); // +/- 250 degrees sec
                WriteByte(AccelConfig, 0); // +/- 2g

                WriteByte(Config, 1); // 184 Hz, 2ms delay
                WriteByte(SmplrtDiv, 19); // set rate 50Hz
                WriteByte(FifoEn, 0x78); // enable accel and gyro to read into fifo
                WriteByte(UserCtrl, 0x40); // reset and enable fifo
                WriteByte(IntEnable, 0x1);
            }
            catch (Exception ex)
            {
                Debug.Fail("An error has occurred when initialising MPU6050: ", ex.ToString());
                throw;
            }
        }

        private void Interrupt(GpioPin sender, GpioPinValueChangedEventArgs args)
        {
            if (m_mpu6050Device == null)
                return;

            try
            {
                int interruptStatus = ReadByte(IntStatus);
                if ((interruptStatus & 0x10) != 0)
                {
                    WriteByte(UserCtrl, 0x44); // reset and enable fifo
                }

                if ((interruptStatus & 0x1) == 0)
                    return;

                var ea = new MpuSensorEventArgs
                {
                    Status = (byte)interruptStatus,
                    SamplePeriod = 0.02f
                };

                var queuedBytes = ReadWord(FifoCount);
                var queuedEvents = queuedBytes / SensorBytes;
                if (queuedEvents == 0)
                    return;

                var events = new MpuSensorValue[queuedEvents];

                for (var eventIndex = 0; eventIndex < queuedEvents; eventIndex++)
                {
                    var data = ReadBytes(FifoRW, SensorBytes);

                    var xa = (short)(data[0] << 8 | data[1]);
                    var ya = (short)(data[2] << 8 | data[3]);
                    var za = (short)(data[4] << 8 | data[5]);

                    var xg = (short)(data[6] << 8 | data[7]);
                    var yg = (short)(data[8] << 8 | data[9]);
                    var zg = (short)(data[10] << 8 | data[11]);

                    events[eventIndex] = new MpuSensorValue
                    {
                        AccelerationX = xa / (float)16384,
                        AccelerationY = ya / (float)16384,
                        AccelerationZ = za / (float)16384,
                        GyroX = xg / (float)131,
                        GyroY = yg / (float)131,
                        GyroZ = zg / (float)131
                    };
                }

                ea.Values = events;

                SensorInterruptEvent?.Invoke(this, ea);
            }
            catch (Exception ex)
            {
                Debug.Fail("An error has occurred when processing interrupt: ", ex.ToString());
                throw;
            }
        }

    }

}
