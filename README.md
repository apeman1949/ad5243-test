# stm32-ad5243-test
必须先写0号再写1号 才能同时控制2个电位器

	I2C_Initializes();
	//EEPROM_WriteByte(0x80,255);
	///*  ok
	EEPROM_WriteByte(0,255);
	delay_ms(1000);delay_ms(1000);
	EEPROM_WriteByte(0,128);
	delay_ms(1000);
	EEPROM_WriteByte(0,64);
	delay_ms(1000);
	EEPROM_WriteByte(0x80,32);
	delay_ms(1000); 
