# Header-only ESP-IDF component for the gateway. The node build adds
# common/protocol/include to its include paths in node/.cproject instead.
idf_component_register(INCLUDE_DIRS "include")
