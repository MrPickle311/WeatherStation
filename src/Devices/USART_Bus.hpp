/*
 * USART_Device.hpp
 *
 *  Created on: 21 Oct 2021
 *      Author: Damian
 */

#ifndef DEVICES_USART_BUS_HPP_
#define DEVICES_USART_BUS_HPP_

namespace Device
{

	class USART_Bus
	{
	public:
		virtual ~USART_Bus() = default;
		USART_Bus(const USART_Bus &other) = delete;
		USART_Bus(USART_Bus &&other) = delete;
		USART_Bus& operator=(const USART_Bus &other) = delete;
		USART_Bus& operator=(USART_Bus &&other) = delete;

	private:
		USART_Bus();
	};

} /* namespace Device */

#endif /* DEVICES_USART_BUS_HPP_ */
