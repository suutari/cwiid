struct dataset{
	struct timespec timestamp;
	uint8_t acc[3];
	uint16_t rate[3];
	uint16_t button;
};

