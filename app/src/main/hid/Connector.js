const HID = require('node-hid');
const BaseConnector = require('../BaseConnector');

class Connector extends BaseConnector {
    open(path) {
        this.connected = false;
        this.device = new HID.HID(path);
        return this.device;
    }

    initialize() {
        return new Promise((resolve, reject) => {
            const { isInitialCheck } = this.options;
            const { requestInitialData, checkInitialData } = this.hwModule;
            if (isInitialCheck && requestInitialData && checkInitialData) {
                this.device.on('data', (data) => {
                    const result = checkInitialData(data, this.options);
                    if (result === undefined) {
                        this.send(requestInitialData());
                    } else {
                        this.device.removeAllListeners('data');
                        if (result === true) {
                            this.connected = true;
                            resolve();
                        } else {
                            reject(new Error('Invalid hardware'));
                        }
                    }
                    console.log(result, data);
                });
                this.device.on('error', reject);
                this.send(requestInitialData());
            } else {
                this.connected = true;
                resolve();
            }
        });
    }

    send(data, type = 'output') {
        if (this.connected) {
            try {
                const writer =
                    type === 'feature'
                        ? this.device.sendFeatureReport
                        : this.device.write;
                return this.device.write(data);
            } catch (e) {
                console.error(e);
            }
        }
    }

    connect() {
        if (!this.router) {
            throw new Error('router must be set');
        }

        if (!this.device) {
            throw new Error('device must be set');
        }

        const router = this.router;
        const hwModule = this.hwModule;

        if (hwModule.connect) {
            hwModule.connect();
        }
        this._sendState('connect');

        if (hwModule.afterConnect) {
            hwModule.afterConnect(this);
        }

        this.device.on('data', (data) => {
            console.log('data', new Buffer(data).toString());
            if (
                !hwModule.validateLocalData ||
                hwModule.validateLocalData(data)
            ) {
                if (!this.connected) {
                    this.connected = true;
                    this._sendState('connected');
                }

                this.received = true;
                let result = data;
                if (hwModule.handleLocalData) {
                    result = hwModule.handleLocalData(result);
                }
                router.sendEncodedDataToServer(result);
            }
        });

        this.device.on('error', (e) => {
            console.log('ERROR', e);
            this.close();
            this._sendState('disconnected');
        });

        // this.send(hwModule.requestInitialData());
        setInterval(() => {
            // const a = hwModule.sensorCheck();
            console.log(Buffer.from('hi$'));
            this.send(Buffer.from('hi$', 'utf-8').toJSON().data);
        }, 3000);
    }

    clear() {
        this.connected = false;
        this.device.removeAllListeners();
    }

    close() {
        this.clear();
        if (this.device) {
            this.device.close();
        }
    }
}

module.exports = Connector;