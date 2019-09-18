const BME280 = require('bme280-sensor');
const MPU9250 = require('mpu9250');
const AHRS = require('ahrs');

function toRadians(degrees) {
  return degrees * (Math.PI / 180);
}

let magXdamp = 0.0;
let magYdamp = 0.0;
function calcTiltAdjustedHeading(magX, magY, magZ, pitch, roll) {
  const magPitch = toRadians(-1 * roll);
  const magRoll = toRadians(pitch);

  const magXhor = magX * Math.cos(magPitch) + magY * Math.sin(magRoll) * Math.sin(magPitch) - magZ * Math.cos(magRoll) * Math.sin(magPitch);
  const magYhor = magY * Math.cos(magRoll) + magZ * Math.sin(magRoll);
  magXdamp = magXdamp * 0.9 + magXhor * 0.1;
  magYdamp = magYdamp * 0.9 + magYhor * 0.1;
  return calcHeading(magYdamp, magXdamp);
}

function calcHeading(x, y) {
  let heading = Math.atan2(y, x) * (180 / Math.PI);
  if (heading > 360) {
    heading -= 360;
  }
  if (heading < 0) {
    heading += 360;
  }
  return heading;
}

module.exports = function plugin(app) {
  let timer = null;
  let lastMag = [0, 0, 0];
  const plugin = {};
  plugin.id = 'signalk-raspberry-pi-gy91';
  plugin.name = 'Raspberry-Pi GY-91';
  plugin.description = 'GY-91 multi-sensor on Raspberry Pi';

  plugin.schema = {
    type: 'object',
    properties: {
      rate: {
        title: 'Sample Rate (in seconds)',
        type: 'number',
        default: 1,
      },
      path: {
        type: 'string',
        title: 'SignalK Path',
        description: 'This is used to build the path in Signal K. It will be appended to \'environment\'',
        default: 'inside.mainCabin',
      },
      i2c_bus: {
        type: 'integer',
        title: 'I2C bus number',
        default: 1,
      },
    },
  };

  plugin.start = function start(options) {
    const bmeOptions = {
      i2cBusNo: options.i2c_bus || 1,
      i2cAddress: 0x76,
    };
    const mpuOptions = {
      device: `/dev/i2c-${options.i2c_bus || 1}`,
      UpMagneto: true,
      scaleValues: true,
      magCalibration: { min: { x: -78.890625, y: -26.3828125, z: -120.25 },
          max: { x: 76.5, y: 123.51953125, z: 40.46875 },
          offset: { x: -1.1953125, y: 48.568359375, z: -39.890625 },
          scale:
             { x: 1.4994846656611363,
                  y: 1.5543843648208469,
                       z: 1.4497739646120942 } },
      /*
      magCalibration: { min: { x: -90.84375, y: -29.98046875, z: -123.71875 },
          max: { x: 62.15625, y: 135.51171875, z: 48.5625 },
          offset: { x: -14.34375, y: 52.765625, z: -37.578125 },
          scale:
           { x: 1.6038347630718954,
                  y: 1.4827692017183591,
                  z: 1.4243379285325595 } },
      gyroBiasOffset: {
        x: 2.316015267175573,
        y: -0.031587786259541964,
        z: -0.23128244274809168
      },
      accelCalibration: {
        offset: {
          x: 0.016557820638020835,
          y: 0.006538492838541667,
          z: 0.35182373046875
        },
        scale: {
          x: [
            0.0403857421875,
            -0.018391927083333332
          ],
          y: [
            -0.0303271484375,
            0.06580485026041667
          ],
          z: [
            0.3269474283854167,
            0.3564152018229167
          ]
        }
      }
      */
    };
    const bme280 = new BME280(bmeOptions);
    const mpu9250 = new MPU9250(mpuOptions);
    const madgwick = new AHRS({
      sampleInterval: 200,
      algorithm: 'Madgwick',
      beta: 0.3,
    });
    let lastReading = new Date();
    let lastSending = new Date();

    function createDeltaMessage(bmeData, mpuData) {
      const madDeg = madgwick.getEulerAnglesDegrees();
      const values = {
        heading: {
          raw: calcHeading(mpuData[6], mpuData[7]),
          til: calcTiltAdjustedHeading(mpuData[6], mpuData[7], mpuData[8], madDeg.pitch, madDeg.roll),
          fix: madDeg.heading,
        },
        pitch: {
          raw: mpu9250.getPitch(mpuData),
          fix: madDeg.pitch,
        },
        roll: {
          raw: mpu9250.getRoll(mpuData),
          fix: madDeg.roll,
        },
      };
      console.log(JSON.stringify(values.heading, null, 2));
      const useValue = 'raw';
      return {
        context: `vessels.${app.selfId}`,
        updates: [
          {
            source: {
              label: plugin.id,
            },
            timestamp: (new Date().toISOString()),
            values: [
              {
                path: `environment.${options.path}.temperature`,
                value: bmeData.temperature_C + 273.15,
              },
              {
                path: `environment.${options.path}.pressure`,
                value: bmeData.pressure_hPa * 100,
              },
              {
                path: 'navigation.headingMagnetic',
                value: toRadians(values.heading['til']),
              },
              {
                path: 'navigation.attitude',
                value: {
                  pitch: toRadians(values.pitch[useValue]),
                  roll: toRadians(values.roll[useValue]),
                  yaw: 0,
                },
              },
            ],
          },
        ],
      };
    }

    function readSensorData() {
      bme280.readSensorData()
        .then((bmeData) => {
          const readingTime = new Date();
          const sinceLastRead = (readingTime.getTime() - lastReading.getTime()) / 1000;
          const sinceLastSend = (readingTime.getTime() - lastSending.getTime()) / 1000;
          let mpuData;
          if (sinceLastRead >= 0.1) {
            // Only get the magnetometer values every 100Hz
            lastReading = readingTime;
            mpuData = mpu9250.getMotion9();
            lastMag = [mpuData[6], mpuData[7], mpuData[8]];
          } else {
            mpuData = mpu9250.getMotion6().concat(lastMag);
          }
          madgwick.update(
            mpuData[3], // gyro
            mpuData[4],
            mpuData[5],
            mpuData[0], // accelerometer
            mpuData[1],
            mpuData[2],
            mpuData[7], // compass, note axis alignment
            mpuData[6],
            mpuData[8] * -1,
          );
          if (sinceLastSend < options.rate) {
            return;
          }
          lastSending = readingTime;
          app.handleMessage(plugin.id, createDeltaMessage(bmeData, mpuData));
        }, (e) => {
          app.error(e);
        });
    }

    bme280.init()
      .then(() => {
        app.setProviderStatus('Connected to BMP280 sensor');
        mpu9250.initialize();
        app.setProviderStatus('Connected to GY-91 multi-sensor');
        readSensorData();
        timer = setInterval(readSensorData, 5);
      }, (e) => {
        app.error(e);
        app.setProviderError(e.message);
      });
  };

  plugin.stop = function stop() {
    if (timer) {
      clearInterval(timer);
      timer = null;
    }
  };

  return plugin;
};
