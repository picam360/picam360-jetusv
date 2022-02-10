var create_plugin = (function() {
	var m_plugin_host = null;
	var m_is_init = false;
	var m_post_map_loaded = null;
	var m_post_map_unloaded = null;
	var m_menu_txt = null;
	var m_popup_callback = null;

	function decodeUtf8(data) {
		var result = "";
		var i = 0;
		var c = 0;
		var c1 = 0;
		var c2 = 0;
		// If we have a BOM skip it
		if (data.length >= 3 && data[0] === 0xef && data[1] === 0xbb
			&& data[2] === 0xbf) {
			i = 3;
		}
		while (i < data.length) {
			c = data[i];

			if (c < 128) {
				result += String.fromCharCode(c);
				i++;
			} else if (c > 191 && c < 224) {
				if (i + 1 >= data.length) {
					throw "UTF-8 Decode failed. Two byte character was truncated.";
				}
				c2 = data[i + 1];
				result += String.fromCharCode(((c & 31) << 6) | (c2 & 63));
				i += 2;
			} else {
				if (i + 2 >= data.length) {
					throw "UTF-8 Decode failed. Multi byte character was truncated.";
				}
				c2 = data[i + 1];
				c3 = data[i + 2];
				result += String.fromCharCode(((c & 15) << 12)
					| ((c2 & 63) << 6) | (c3 & 63));
				i += 3;
			}
		}
		return result;
	}
	function addMenuButton(name, txt) {
			return new Promise((resolve, reject) => {
			var onsListItem = document.createElement("ons-list-item");
			onsListItem.id = name;
			onsListItem.innerHTML = txt;
			menu_list.insertBefore(onsListItem, menu_list_about);
			ons.compile(onsListItem);
			resolve();
		});
	}
	function map_load() {
		m_container.style.display = "block";
		m_overlay = new ol.Overlay({
			element : m_container,
			autoPan : true,
			autoPanAnimation : {
				duration : 250
			}
		});
		var map = new ol.Map({
			target : 'map',
			layers : [new ol.layer.Tile({
				source : new ol.source.OSM()
			})],
			overlays : [m_overlay],
			view : new ol.View({
				center : ol.proj.fromLonLat([136.1228505, 35.2937157]),
				zoom : 8
			})
		});
		var menu_swiping = false;
		var mousedownFunc = function(ev) {
			if (ev.type == "touchstart") {
				ev.clientX = ev.pageX;
				ev.clientY = ev.pageY;
			}
			menu_swiping = (ev.clientX < 100);
		};
		document.addEventListener("touchstart", mousedownFunc);
		document.addEventListener("mousedown", mousedownFunc);
		map.addEventListener("pointerdrag", function(ev) {
			if (menu_swiping) {
				ev.preventDefault();
			}
		});

		if (m_post_map_loaded) {
			m_post_map_loaded(map);
		}
	}
	function map_unload() {
		if (m_post_map_unloaded) {
			m_post_map_unloaded();
		}
	}
	function init(plugin) {
		console.log(plugin.name + "::init()");

		var script = document.createElement('script');
		script.src = "http://localhost/openlayers-v5.1.3/ol.js";
		script.onload = function() {
			m_plugin_host
				.getFile("plugins/map/map.html", function(chunk_array) {
					var txt = decodeUtf8(chunk_array[0]);
					var node = $.parseHTML(txt);
					$('body').append(node);
					var map_page = document.getElementById("map_page");
					ons.compile(node[0]);

					app.navi.on('postpush', function(event) {
						if (event.enterPage.name == "map.html") {
							map_load();
						}
					});

					app.navi.on('postpop', function(event) {
						if (event.leavePage.name == "map.html") {
							if (m_popup_callback) {
								m_popup_callback();
								m_popup_callback = null;
							}
							map_unload();
						}
					});
				});
			m_plugin_host.getFile("plugins/map/popup.html", function(
				chunk_array) {
				var txt = decodeUtf8(chunk_array[0]);
				var node = $.parseHTML(txt);
				$('body').append(node);
				m_container = document.getElementById('popup');
				m_content = document.getElementById('popup-content');
				m_closer = document.getElementById('popup-closer');
				m_closer.onclick = function() {
					if (m_overlay) {
						m_overlay.setPosition(undefined);
					}
					if (m_popup_callback) {
						m_popup_callback();
						m_popup_callback = null;
					}
					m_closer.blur();
					return false;
				};
			});
		};
		document.head.appendChild(script);
	}
	return function(plugin_host) {
		m_plugin_host = plugin_host;
		var plugin = {
			name : "map",
			init_options : function(options) {
				if (!m_is_init) {
					m_is_init = true;
					init(plugin);
				}
			},
			event_handler : function(sender, event) {

			},
			set_post_map_loaded : function(callback) {
				m_post_map_loaded = callback;
			},
			set_post_map_unloaded : function(callback) {
				m_post_map_unloaded = callback;
			},
			on_restore_app_menu : function(callback) {
				addMenuButton("swMap", "Map").then(() => {
					swMap.onclick = (evt) => {
						app.navi.pushPage('map.html');
						app.menu.closeMenu();
					};
				});
			},
			popup : function(coordinate, msg, callback) {
				m_content.innerHTML = msg;
				if (m_popup_callback) {
					m_popup_callback();
					m_popup_callback = null;
				}
				m_popup_callback = callback;
				if (m_overlay) {
					m_overlay.setPosition(coordinate);
				}
			},
			set_popup_msg : function(msg) {
				m_content.innerHTML = msg;
			},
		};
		return plugin;
	}
})();