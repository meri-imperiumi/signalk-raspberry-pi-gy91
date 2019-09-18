const BME280 = require('bme280-sensor');
const MPU9250 = require('mpu9250');

function calcHeading(x, y) {
    var heading = Math.atan2(y, x) * 180 / Math.PI;
    if (heading < 0) {
      heading += 2*Math.PI
    }
    /*
    if (heading < -180) {
        heading += 360;
    } else if (heading > 180) {
        heading -= 360;
    }
    */

    return heading;
}

module.exports = function plugin(app) {
  let timer = null;
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
      magCalibration: { min: { x: -87.2578125, y: -35.9765625, z: -126.03125 },
          max: { x: 105.1875, y: 115.125, z: 75.15625 },
          offset: { x: 8.96484375, y: 39.57421875, z: -25.4375 },
          scale:
           { x: 1.4152965534039703,
                  y: 1.8025438188304639,
                  z: 1.3537977632805218 } },
      DEBUG: true,
    };
    const bme280 = new BME280(bmeOptions);
    const mpu9250 = new MPU9250(mpuOptions);

    function createDeltaMessage(bmeData, mpuData) {
      console.log(`Gyro x: ${mpuData[3]} y: ${mpuData[4]} z: ${mpuData[5]}`);
      console.log(`Compass x: ${mpuData[6]} y: ${mpuData[7]} z: ${mpuData[8]}`);
      console.log(calcHeading(mpuData[6], mpuData[7]));
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
                value: Math.atan2(mpuData[6], mpuData[7]),
              },
            ],
          },
        ],
      };
    }

    function readSensorData() {
      bme280.readSensorData()
        .then((bmeData) => {
          const mpuData = mpu9250.getMotion9();
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
        timer = setInterval(readSensorData, options.rate * 1000);
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