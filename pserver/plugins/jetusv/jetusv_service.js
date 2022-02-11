module.exports = {
	create_plugin: function (plugin_host) {
		console.log("create jetusv plugin");
		var async = require('async');
		var fs = require("fs");
		var sprintf = require('sprintf-js').sprintf;
		var SerialPort = require('serialport');
		var gpsd = require('node-gpsd');
		var child_process = require('child_process');
		var i2c_buf = require("i2c-bus");
		var Pca9685Driver = require("pca9685").Pca9685Driver;
		var Ads1x15 = require('node-ads1x15');
		var usvd = require('./node-usvd');

		var PLUGIN_NAME = "jetusv";
		var DELIMITER = '\r\n';
		var TIMEOUT_MS = 5000;
		var ADS7828_ADDRESS = 0x48;
		var START_TIME = Date.now();

		var m_gps_valid = false;
		var m_latitude = 0;
		var m_longitude = 0;
		var m_speed_kmph = 0;
		var m_north = 0;
		var m_rudder = 0;
		var m_thruster = 0;
		var m_pid_enabled = 0;
		var m_adc_values = [];
		var m_battery = 0;
		var m_battery_sens = 0;
		var m_solar_sens = 0;
		var m_battery_amp = 0;
		var m_solar_amp = 0;

		// downstream status
		var m_waypoints_required = false;
		var m_history_required = false;

		// propo
		var m_ch_us = [];

		// manual
		var m_manual_target_heading = 0;
		var m_manual_thruster = 0;

		// auto
		var m_next_waypoint_distance = 0;
		var m_target_heading = 0;
		var m_operator_pos = {};

		var options = {
			thruster_mode: "SINGLE",
			// auto
			propo_enabled: false,
			manual_enabled: true,
			manual_pid_enabled: false,
			operation_mode: "STANBY",
			waypoints: [],
			history: {
				history_10000min: [],// around 6days
				history_1000min: [],
				history_100min: [],
				history_10min: [],
				history_1min: [],
			},
			next_waypoint_idx: 0,
			gain_kp: 160,
			gain_kv: 40,
			low_gain_kp: 40,
			low_gain_kv: 10,
			low_gain_deg: 5,
			SKREW_PINS: [0, 1, 2, 3],
			PWM_MIN_US: 1300,
			PWM_MIDDLE_US: 1500,
			PWM_MAX_US: 1700,
			PWM_MARGIN_MS: 100,
			ch: [{}, {}, {}, {}],
		};

		// aws_iot
		var clientTokenUpdate;
		var clientTokenPublish;
		var clientTokenGet;
		var clientTokenGetCallback;

		function toFixedFloat(value, c) {
			return parseFloat(value ? value.toFixed(c) : 0);
		}

		function get_status() {
			var status = {
				gps: m_gps_valid,
				lat: toFixedFloat(m_latitude, 6),
				lon: toFixedFloat(m_longitude, 6),
				spd: toFixedFloat(m_speed_kmph, 3),
				heading: toFixedFloat(-m_north, 3), // heading_from_north_clockwise
				bat: toFixedFloat(m_battery, 3),
				bamp: toFixedFloat(m_battery_amp, 3),
				samp: toFixedFloat(m_solar_amp, 3),
				adc: m_adc_values,
				next_waypoint_idx: options.next_waypoint_idx,
				next_waypoint_distance: m_next_waypoint_distance,
				rudder: toFixedFloat(m_rudder, 0),
				thruster: toFixedFloat(m_thruster, 0),
				operation_mode: options.operation_mode,
				gain_kp: options.gain_kp,
				gain_kv: options.gain_kv,
				low_gain_kp: options.low_gain_kp,
				low_gain_kv: options.low_gain_kv,
				low_gain_deg: options.low_gain_deg,
				home: options.home,
				operator_pos: m_operator_pos,
				manual_pid_enabled: options.manual_pid_enabled,
			};

			return status;
		}

		var set_thruster_pwm = function(idx, us){
			//implement pwm block
		};

		var on_north_received = function(quat){
		};

		async.waterfall([
			function (callback) {
				//9axis sensor handler
				console.log("init 9axis handler");

				var def = "dummy s=2x2 fps=10 ! icm20948 mode=yaw";
				var pst = plugin_host.pstcore.pstcore_build_pstreamer(def);

                var meta = "<meta maptype=\"DUMMY\" deg_offset=\"-90,0,0\" />";
				plugin_host.pstcore.pstcore_set_param(pst, "dummy", "meta", meta);

				var buff = [];
				plugin_host.pstcore.pstcore_set_dequeue_callback(pst, (data)=>{
					if(data == null){//eob
						var split = buff[1].toString().split(' ');
						for (var i = 0; i < split.length; i++) {
							var separator = (/[=,\"]/);
							var _split = split[i].split(separator);
							if (_split[0] == "vehicle_quat") {
								var quat = [parseFloat(_split[2]),
									parseFloat(_split[3]),
									parseFloat(_split[4]),
									parseFloat(_split[5])
								];
								m_north = quat[1] * 180;
								on_north_received(m_north);
							}
						}
						buff = [];
						return;
					}
					buff.push(data);
				});
				plugin_host.pstcore.pstcore_start_pstreamer(pst);

				callback(null);
				//9axis sensor handler
			},
			function (callback) {
				//pwm handler
				console.log("init pwm handler");
				
				var pwm_options = {
					i2c: i2c_buf.openSync(1),
					address: 0x40,
					frequency: 50,
					debug: false
				};
				var pwm = new Pca9685Driver(pwm_options, function(err) {
					if (err) {
						console.error("Error initializing PCA9685");
						process.exit(-1);
					}
					console.log("Initialization done");

					for(var idx in options.SKREW_PINS){
						pwm.setPulseLength(options.SKREW_PINS[idx], 1500);
					}
					process.on('SIGINT', function() {
						pwm.setPulseLength(0, 1500);
						pwm.setPulseLength(1, 1500);
						process.exit();
					});

					//implemtn module scope function
					set_thruster_pwm = function(idx, us){
						pwm.setPulseLength(options.SKREW_PINS[idx], us);
					};

					callback(null);
				});
				//pwm handler
			},
			function (callback) {
				//adc handler
				console.log("init adc handler");
				var chip = 0; //0 for ads1015, 1 for ads1115  
				var adc = new Ads1x15(chip, 0x48, '/dev/i2c-1')

				var channel = 0;
				var samplesPerSecond = '250'; // see index.js for allowed values for your chip  
				var progGainAmp = '4096'; // see index.js for allowed values for your chip 

				m_adc_values = [];

				function update_adc_values(callback) {
					adc.readADCSingleEnded(channel, progGainAmp, samplesPerSecond, function(err, data) {   
						if(err)  
						{
							setTimeout(() => {
								update_adc_values(callback);
							}, 20);
							return;
						}
						m_adc_values[channel] = data;

						channel++;
						if(channel < 4){
							update_adc_values(callback);
						}else{
							if(callback){
								callback();
							}
							channel = 0;
							setTimeout(() => {
								update_adc_values(callback);
							}, 200);
						}
					});
				};
				
				update_adc_values(() => {
					if (options.adc_debug) {
						var msg = "adc";
						for (var i = 0; i < 16; i++) {
							msg += " ch" + i + " : " + m_adc_values[i];
						}
						console.log(msg);
					}
					if (options.operation_mode == "WAYPOINT"
						&& options.adc_log) {
						function isExistFile(file) {
							try {
								fs.statSync(file);
								return true
							} catch (err) {
								if (err.code === 'ENOENT')
									return false
							}
						}
						if (!isExistFile("adc_log")) {
							var msg = "t,lat,lon,head";
							for (var i = 0; i < 16; i++) {
								msg += ",ch" + i;
							}
							child_process.execSync('echo "' + msg
								+ '" >> adc_log');
						}

						var msg = parseInt(Date.now() / 1000) + ","
							+ m_latitude + "," + m_longitude + ","
							+ (-m_north);
						for (var i = 0; i < 16; i++) {
							msg += "," + m_adc_values[i];
						}
						child_process.execSync('echo "' + msg
							+ '" >> adc_log');
					}
					{// battery
						var lpf_gain = 0.02;
						var battery_sens = 6 * 2.5 * m_adc_values[0]
							/ (1 << 12);
						var solar_sens = 6 * 2.5 * m_adc_values[1]
							/ (1 << 12);
						var out_v = 6 * 2.5 * m_adc_values[2] / (1 << 12);
						if (m_battery_sens < 5) {
							m_battery_sens = battery_sens;
						} else {
							m_battery_sens = battery_sens * lpf_gain
								+ m_battery_sens * (1.0 - lpf_gain);
						}
						if (m_solar_sens < 5) {
							m_solar_sens = out_v;
						} else {
							m_solar_sens = solar_sens * lpf_gain
								+ m_solar_sens * (1.0 - lpf_gain);
						}
						if (m_battery < 5) {
							m_battery = out_v;
						} else {
							m_battery = out_v * lpf_gain + m_battery
								* (1.0 - lpf_gain);
						}
						m_battery_amp = (m_battery_sens - m_battery) / 0.01
							- (options.bamp_offset || 0);
						m_solar_amp = (m_solar_sens - m_battery) / 0.01
							- (options.samp_offset || 0);
					}
					if (options.propo_enabled) { // propo
						var debug_str = "";
						for (var i = 1; i <= 3; i++) {
							var ch_v = 2.5 * m_adc_values[i] / (1 << 12);
							var ch_us = ch_v / 4 * 20000;

							if (m_ch_us[i] === undefined) {
								var mid = (options.propo[i] && options.propo[i].PWM_MIDDLE_US)
									|| options.PWM_MIDDLE_US;
								if (mid - 50 < ch_us && ch_us < mid + 50) {
									m_ch_us[i] = mid;
								}
							} else {
								m_ch_us[i] = ((m_ch_us[i] * 9) + ch_us) / 10;
							}
							if (options.propo_debug) {
								debug_str += "ch"
									+ i
									+ "="
									+ ch_v.toFixed(3)
									+ "V,"
									+ ch_us.toFixed()
									+ "us,"
									+ (m_ch_us[i]
										? m_ch_us[i].toFixed()
										: "****") + "us;";
							}
						}
						if (options.propo_debug) {
							console.log(debug_str);
						}
					}
				});
				callback(null);
				//adc handler
			},
			function (callback) {
				//gps handler
				var listener = new gpsd.Listener({
					port: 2947,
					hostname: 'localhost',
					logger: {
						info: function () {
						},
						warn: console.warn,
						error: console.error
					},
					parse: true
				});
				listener.connect(function () {
					console.log('GPSD Connected');
					listener.on('TPV', function (tpvData) {
						if (options.gps_debug) {
							if (options.gps_test) {
								tpvData = options.gps_test;
							}
							console.log(tpvData);
						}
						if (tpvData.lat && tpvData.lon) {
							// speed
							var earth_r_km = 6356.752;
							var equator_r_km = 6378.137;
							var lat_deg2m = earth_r_km * Math.PI / 180
								* 1000;
							var lon_deg2m = equator_r_km
								* Math.cos(m_latitude * Math.PI / 180)
								* Math.PI / 180 * 1000;
							var d_lat_deg = tpvData.lat - m_latitude;
							var d_lon_deg = tpvData.lon - m_longitude;
							var d_lat_m = d_lat_deg * lat_deg2m;
							var d_lon_m = d_lon_deg * lon_deg2m;
							var speed_kmph = Math.sqrt(d_lat_m * d_lat_m
								+ d_lon_m * d_lon_m) / 1000 * 3600;
							if (m_gps_valid && speed_kmph < 100) {
								var lpf_gain = 0.2;
								m_speed_kmph = speed_kmph * lpf_gain
									+ m_speed_kmph * (1.0 - lpf_gain);
							}

							// update
							m_latitude = tpvData.lat;
							m_longitude = tpvData.lon;
							m_gps_valid = true;
						} else { // GPS_LOST
							m_gps_valid = false;
						}
					});
					listener.watch();
				});
				// listener.disconnect(function() {
				// console.log('Disconnected');
				// });
				callback(null);
				//gps handler
			},
			function (callback) {
				//history handler
				setInterval(function () {
					var state = get_status();
					// update history
					// care shadow size limited 8k
					var history_tbl = options.history;
					var max_count_tbl = [5, 5, 5, 5, 5];// max25nodes30days
					var interval_s_tbl = [60, 60 * 10, 60 * 100,
						60 * 1000, 60 * 10000];
					var new_key = parseInt(Date.now() / 1000);
					var new_value;
					if (!m_gps_valid) { // GPS_LOST
						new_value = {
							"gps": false,
							"bat": state.bat,
						};
					} else {
						new_value = {
							"lat": state.lat,
							"lon": state.lon,
							"bat": state.bat,
						};
					}
					for (var i = 0; i < history_tbl.length - 1; i++) {
						var keys = Object.keys(history_tbl[i]);
						for (var j = 0; j < keys.length
							- max_count_tbl[i]; j++) {
							delete history_tbl[i][keys[j]];
						}
					}
					for (var i = 0; i < history_tbl.length - 1; i++) {
						var keys = Object.keys(history_tbl[i]);
						if (new_key - (keys[keys.length - 1] || 0) > interval_s_tbl[i]) {
							history_tbl[i][new_key] = new_value;
						} else {
							break;
						}
						if (keys.length + 1 > max_count_tbl[i]) {
							new_key = keys[0];
							new_value = history_tbl[i][new_key];
							delete history_tbl[i][new_key];
						} else {
							break;
						}
					}
					plugin.save_json("history.json", history_tbl);
				}, 60 * 1000);
				callback(null);
				//history handler
			},
			function (callback) {
				//usvd handler
				console.log("init usvd handler");
				usvd.usvd_init("", (ch, us) => {
					set_thruster_pwm(ch, us);
					//console.log(ch, us);
				});
				on_north_received = function(north){
					var t_ms = Date.now() - START_TIME;
					usvd.usvd_poll(t_ms/1000, north);
				};
				// pwm output
				function get_pwm_us(value_per, ch_options) {
					var mid = (ch_options && ch_options.PWM_MIDDLE_US)
						|| options.PWM_MIDDLE_US;
					var min = (ch_options && ch_options.PWM_MIN_US)
						|| options.PWM_MIN_US;
					var max = (ch_options && ch_options.PWM_MAX_US)
						|| options.PWM_MAX_US;
					var inv = ch_options && ch_options.invert;
					var value = (inv ? -1 : 1) * value_per
						* (max - min) + mid;
					return Math.max(min, Math.min(value, max));
				}
				function get_per_from_pwm(value_us, ch_options) {
					var mid = (ch_options && ch_options.PWM_MIDDLE_US)
						|| options.PWM_MIDDLE_US;
					var min = (ch_options && ch_options.PWM_MIN_US)
						|| options.PWM_MIN_US;
					var max = (ch_options && ch_options.PWM_MAX_US)
						|| options.PWM_MAX_US;
					var inv = ch_options && ch_options.invert;
					var value = (inv ? -1 : 1) * (value_us - mid)
						/ (max - min) * 100;
					return Math.max(-100, Math.min(value, 100));
				}
				setInterval(function () {
					if (options.operation_mode == "MANUAL"||
						options.operation_mode == "EMERGENCY") {
						if (options.propo_enabled && 1500 < m_ch_us[3]
							&& m_ch_us[3] < 2000) {
							m_thruster = get_per_from_pwm(m_ch_us[2], options.propo[2]);
						} else if (options.manual_enabled) {
							m_thruster = m_manual_thruster;
							m_target_heading = m_manual_target_heading;
							if (options.manual_debug) {
								console.log("manual : rud "
									+ m_manual_rudder + " : thr "
									+ m_manual_thruster);
							}
						} else {
							m_thruster = 0;
						}
					}
					switch (options.operation_mode) {
						case "WAYPOINT" :
						case "HOME" :
						case "FOLLOW" : {
							var cmd = "set_thrust "
								+ m_thruster + "," + m_rudder + ","
								+ m_target_heading + ","
								+ (m_pid_enabled ? 1 : 0);
							usvd.usvd_command(cmd);
							break;
						}
						case "MANUAL" :
						case "EMERGENCY" : {
							var cmd = "set_thrust "
								+ m_thruster + "," + m_rudder + ","
								+ m_target_heading + ","
								+ (options.manual_pid_enabled ? 1 : 0);
							usvd.usvd_command(cmd);
							//console.log(cmd);
							break;
						}
						case "STANBY" :
						default : {
							usvd.usvd_command("set_thrust 0,0,0,0");
							break;
						}
					}
				}, 200);
				callback(null);
				//usvd handler
			},
			function (callback) {
				// auto operation
				var waypoint_data = null;
				setInterval(function () {
					if (options.operation_mode == "WAYPOINT"
						|| options.operation_mode == "HOME"
						|| options.operation_mode == "FOLLOW") {
					} else {
						return;
					}
					// reset pwm
					m_thruster = 0;
					if (!m_gps_valid) { // GPS_LOST
						return;
					}
					var waypoint;
					switch (options.operation_mode) {
						case "WAYPOINT": {
							// back to start
							if (options.next_waypoint_idx >= options.waypoints.length) {
								options.next_waypoint_idx = 0;
							}
							if (!options.waypoints[options.next_waypoint_idx]) {
								return;
							}
							waypoint = options.waypoints[options.next_waypoint_idx];
							break;
						}
						case "HOME":
							if (!options.home) {
								return;
							}
							waypoint = options.home;
							break;
						case "FOLLOW":
							waypoint = m_operator_pos;
							break;
					}

					var earth_r_km = 6356.752;
					var equator_r_km = 6378.137;
					var lat_deg2m = earth_r_km * Math.PI / 180 * 1000;
					var lon_deg2m = equator_r_km
						* Math.cos(m_latitude * Math.PI / 180) * Math.PI
						/ 180 * 1000;
					var d_lat_deg = waypoint.lat - m_latitude;
					var d_lon_deg = waypoint.lon - m_longitude;
					var d_lat_m = d_lat_deg * lat_deg2m;
					var d_lon_m = d_lon_deg * lon_deg2m;

					m_next_waypoint_distance = Math.sqrt(d_lat_m * d_lat_m
						+ d_lon_m * d_lon_m);
					m_target_heading = Math.atan2(d_lon_m, d_lat_m) * 180
						/ Math.PI; // direction_from_north_clockwise
					if (m_target_heading < -180) {
						m_target_heading += 2 * 180;
					}
					if (m_target_heading > 180) {
						m_target_heading -= 2 * 180;
					}

					switch (options.operation_mode) {
						case "WAYPOINT":
							if (Math.abs(m_next_waypoint_distance) < (waypoint.tol || 5)) { // tol_is_Tolerance
								// arrived
								if (options.auto_debug) {
									console.log("arrived at : "
										+ options.next_waypoint_idx);
								}
								if (!waypoint_data) {
									waypoint_data = {
										cmds: []
									};
								}
								if (waypoint.cmds) {
									function isObject(o) {
										return (o instanceof Object && !(o instanceof Array))
											? true
											: false;
									};
									var wait = function (data, params) {
										if (!isObject(params)) {
											params = {
												value: params
											};
										}
										if (!data.start) {
											data.start = params.start
												|| Date.now() / 1000;
										}
										return (Date.now() / 1000 > data.start
											+ params.value);
									}
									var sampling = function (data, params) {
										if (!isObject(params)) {
											params = {
												ch: params,
												cnt: 10
											};
										}
										var value = get_ads7828_value(params.ch, params.cnt);
										var state = {
											"lat": m_latitude,
											"lon": m_longitude,
											"value": value,
										};
										if (plugin.aws_thing_shadow) {
											plugin
												.aws_iot_publish(plugin.aws_thing_shadow, plugin.aws_client_id, params.topic, state);
										}
										return true;
									}
									var funcs = {
										"wait": wait,
										"sampling": sampling,
									};
									var cmds = waypoint.cmds;
									if (!Array.isArray(cmds)) {
										cmds = [cmds];
									}
									for (var i = 0; i < cmds.length; i++) {
										var func = funcs[cmds[i].func];
										if (!func) {
											// error
											continue;
										}
										if (!waypoint_data.cmds[i]) {
											waypoint_data.cmds[i] = {};
										}
										var ret = func(waypoint_data.cmds[i], cmds[i].params);
										if (!ret) {
											return;
										} else {
											continue;
										}
									}
								}
								options.next_waypoint_idx++;
								waypoint_data = null;
								return;
							} else {
								m_pid_enabled = true;
								m_thruster = 100;
							}
							break;
						case "HOME":
						case "FOLLOW":
							if (Math.abs(m_next_waypoint_distance) < (waypoint.tol || 5)) {
								m_thruster = 50;
								// power save
								if (!m_pid_enabled
									|| Math.abs(m_next_waypoint_distance) < (waypoint.tol || 5) / 2) {
									m_pid_enabled = false;
									m_thruster = 0;
								}
							} else {
								m_pid_enabled = true;
								m_thruster = 100;
							}
							break;
					}
				}, 200);
				callback(null);
				// auto operation
			}], function (err, result) {
			});//end of async
		var plugin = {
			name: PLUGIN_NAME,
			aws_thing_shadow: null,
			aws_client_id: null,
			pst_params: {},
			init_options: function (_options) {
				options = Object.assign(options, _options.jetusv);
				options.waypoints = this.load_json("waypoints.json", []);
				options.history = this.load_json("history.json", {
					history_10000min: [],
					history_1000min: [],
					history_100min: [],
					history_10min: [],
					history_1min: [],
				});
			},
			load_json(filepath, dvalue) {
				if (!fs.existsSync(filepath)) {
					return dvalue;
				}
				var lines = fs.readFileSync(filepath, 'utf-8').replace(/\r/g, '').split('\n')
				for (var i = 0; i < lines.length; i++) {
					if (lines[i][0] == '#') {
						lines[i] = "";
					}
				}
				var json_str = lines.join("\n");
				return JSON.parse(json_str);
			},
			save_json(filepath, value) {
				var json_str = JSON.stringify(value);
				fs.writeFileSync(filepath, json_str);
			},
			pst_started: function (pstcore, pst) {
				this.pst_params[pst] = {};
				this.pst_params[pst].timer = setInterval(function () {
					var status = get_status();
					if (m_waypoints_required) {
						status.waypoints = options.waypoints;
						m_waypoints_required = false
					}
					if (m_history_required) {
						status.history = Object
							.assign({}, options.history);
						m_history_required = false
					}
					pstcore.pstcore_set_param(pst, "jetusv", "status", Buffer.from(JSON.stringify(status)).toString('base64'));
				});
				pstcore.pstcore_add_set_param_done_callback(pst, (msg) => {
					//console.log(msg);
				});
			},
			pst_stopped: function (pstcore, pst) {
				clearInterval(this.pst_params[pst].timer);
				delete this.pst_params[pst];
			},
			command_handler: function (cmd) {
				var split = cmd.split(' ');
				switch (split[0]) {
					case "set_waypoints":
						var json_str = decodeURIComponent(split[1]);
						var new_waypoints = JSON.parse(json_str);
						console.log(new_waypoints);
						options.waypoints = new_waypoints;
						plugin.save_json("waypoints.json", options.waypoints);
						if (plugin.aws_thing_shadow) {
							plugin
								.aws_iot_update(plugin.aws_thing_shadow, plugin.aws_client_id, {
									waypoints: options.waypoints
								});
						}
						break;
					case "set_next_waypoint_idx":
						var v = parseInt(split[1]);
						options.next_waypoint_idx = v;
						break;
					case "set_operation_mode":
						usvd.usvd_command("set_emergency_mode 0");
						switch (split[1].toUpperCase()) {
							case "WAYPOINT":
								options.operation_mode = "WAYPOINT";
								break;
							case "HOME":
								options.operation_mode = "HOME";
								break;
							case "FOLLOW":
								options.operation_mode = "FOLLOW";
								break;
							case "MANUAL":
								options.operation_mode = "MANUAL";
								break;
							case "EMERGENCY":
								options.operation_mode = "EMERGENCY";
								usvd.usvd_command("set_emergency_mode 1");
								break;
							case "STANBY":
							default:
								options.operation_mode = "STANBY";
								break;
						}
						break;
					case "set_operator_position":
						m_operator_pos = {
							lon: parseFloat(split[1]),
							lat: parseFloat(split[2]),
							tol: parseFloat(split[3]),
						};
						break;
					case "set_thruster":
						var range = (options.PWM_MAX_US - options.PWM_MIN_US) / 2;
						if (split.length > 1 && !isNaN(split[1])) {
							m_manual_thruster = Math.max(-100, Math
								.min(parseFloat(split[1]), 100));
						}
						if (split.length > 2 && !isNaN(split[2])) {
							if(options.manual_pid_enabled){
								m_manual_target_heading = m_target_heading
									+ split[2] / 100 * 45;
								if (m_manual_target_heading < -180) {
									m_manual_target_heading += 2 * 180;
								}
								if (m_manual_target_heading > 180) {
									m_manual_target_heading -= 2 * 180;
								}
							}else{
								m_rudder = Math.max(-100, Math
									.min(parseFloat(split[2]), 100));
							}
						}
						break;
					case "get_waypoints":
						m_waypoints_required = true;
						break;
					case "get_history":
						m_history_required = true;
						break;
				}
			},
			aws_iot_conneced: function (thing_shadow, client_id) {
				this.aws_thing_shadow = thing_shadow;
				this.aws_client_id = client_id;
				var reported_fnc = function (state, is_delta) {
					var report = {};
					if (state.lat !== undefined) {
						m_latitude = state.lat;
						report.lat = m_latitude;
					}
					if (state.lon !== undefined) {
						m_longitude = state.lon;
						report.lon = m_longitude;
					}
					if (state.waypoints !== undefined) {
						options.waypoints = state.waypoints;
						report.waypoints = options.waypoints;
					}
					if (state.thruster_mode !== undefined) {
						options.thruster_mode = state.thruster_mode;
						report.thruster_mode = options.thruster_mode;

						//TODO:
						// // reset
						// set_thruster_pwm(0, options.PWM_MIDDLE_US, fd);
						// set_thruster_pwm(1, options.PWM_MIDDLE_US, fd);
						// set_thruster_pwm(2, options.PWM_MIDDLE_US, fd);
						// set_thruster_pwm(3, options.PWM_MIDDLE_US, fd);
					}
					if (state.next_waypoint_idx !== undefined) {
						options.next_waypoint_idx = state.next_waypoint_idx;
						report.next_waypoint_idx = options.next_waypoint_idx;
					}
					if (state.operation_mode !== undefined) {
						options.operation_mode = state.operation_mode;
						report.operation_mode = options.operation_mode;
					}
					if (state.gain_kp !== undefined) {
						options.gain_kp = state.gain_kp;
						report.gain_kp = options.gain_kp;
					}
					if (state.gain_kv !== undefined) {
						options.gain_kv = state.gain_kv;
						report.gain_kv = options.gain_kv;
					}
					if (state.low_gain_kp !== undefined) {
						options.low_gain_kp = state.low_gain_kp;
						report.low_gain_kp = options.low_gain_kp;
					}
					if (state.low_gain_kv !== undefined) {
						options.low_gain_kv = state.low_gain_kv;
						report.low_gain_kv = options.low_gain_kv;
					}
					if (state.low_gain_deg !== undefined) {
						options.low_gain_deg = state.low_gain_deg;
						report.low_gain_deg = options.low_gain_deg;
					}
					return report;
				}
				var delta_fnc = function (state) {
					var report = reported_fnc(state, true);
					var cmd = {
						"state": {
							"desired": null,
							"reported": report
						}
					};
					clientTokenUpdate = thing_shadow.update(client_id, cmd);
				}
				thing_shadow
					.on('status', function (thingName, stat, clientToken,
						stateObject) {
						if (stat == 'rejected') {
							console.log(clientToken + " rejected : "
								+ stateObject);
							return;
						}
						if (clientToken == clientTokenPublish) {
							console.log(clientToken + " puglish : " + stat
								+ " : " + stateObject);
						}
						if (clientToken == clientTokenGet) {
							if (stateObject.state.reported) {
								var state = stateObject.state.reported;
								reported_fnc(state, false);

								if (state.history_1min) {
									options.history.history_1min = state.history_1min;
								}
								if (state.history_10min) {
									options.history.history_10min = state.history_10min;
								}
								if (state.history_100min) {
									options.history.history_100min = state.history_100min;
								}
								if (state.history_1000min) {
									options.history.history_1000min = state.history_1000min;
								}
								if (state.history_10000min) {
									options.history.history_10000min = state.history_10000min;
								}
							}
							if (stateObject.state.delta) {
								delta_fnc(stateObject.state.delta);
							}
							if (clientTokenGetCallback) {
								clientTokenGetCallback();
								clientTokenGetCallback = null;
							}
						} // end of get
						if (options.aws_iot_debug) {
							console.log('received ' + stat + ' on ' + thingName
								+ ':' + JSON.stringify(stateObject));
						}
					});
				thing_shadow.on('delta', function (thingName, stateObject) {
					delta_fnc(stateObject.state);
				});
				thing_shadow.on('timeout', function (thingName, clientToken) {
					if (clientToken == clientTokenGet) {
						clientTokenGet = thing_shadow.get(client_id);
					}
					console.log('received timeout : ' + clientToken);
				});
			},
			aws_iot_registered: function (thing_shadow, client_id) {
				this
					.aws_iot_get(thing_shadow, client_id, function () {
						// start sync
						setInterval(function () {
							state.history_1min = options.history.history_1min;
							state.history_10min = options.history.history_10min;
							state.history_100min = options.history.history_100min;
							state.history_1000min = options.history.history_1000min;
							state.history_10000min = options.history.history_10000min;

							plugin
								.aws_iot_update(thing_shadow, client_id, state);
						}, (options.aws_iot_interval_sec || 10) * 1000);
					});
			},
			aws_iot_get: function (thing_shadow, client_id, callback) {
				clientTokenGetCallback = callback;
				clientTokenGet = thing_shadow.get(client_id);
			},
			aws_iot_update: function (thing_shadow, client_id, state) {
				var cmd = {
					"state": {
						"reported": state
					}
				};
				clientTokenUpdate = thing_shadow.update(client_id, cmd);
				if (options.aws_iot_debug) {
					console.log("report shadow: " + JSON.stringify(cmd));
				}
			},
			aws_iot_publish: function (thing_shadow, client_id, topic, state) {
				var message = JSON.stringify(state);
				if (options.aws_iot_debug) {
					console.log("publish: " + message);
				}
				clientTokenPublish = thing_shadow.publish(topic, message);
			},
		};
		return plugin;
	}
};