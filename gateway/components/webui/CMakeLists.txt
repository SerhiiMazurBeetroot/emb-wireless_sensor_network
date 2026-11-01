idf_component_register(
	SRCS "webui.c"
	INCLUDE_DIRS "include"
	EMBED_FILES "web_page.html"
	REQUIRES
		esp_http_server
		esp_wifi
		esp_netif
		esp_event
		esp_timer
		freertos
		log
		protocol
)
