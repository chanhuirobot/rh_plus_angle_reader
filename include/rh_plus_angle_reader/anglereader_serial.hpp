
#ifndef ANGLEREADER_SERIAL__H
#define ANGLEREADER_SERIAL__H
#include <hidapi/hidapi.h>
#include <sstream>
#include <vector>

#include "anglereader_drvr.hpp"

namespace anglereader
{
	class anglereader_serial: public anglereader_drvr
	{
		public:
			anglereader_serial();
			~anglereader_serial();

			bool open(const std::string &portname) override;
			void close() override;

			bool getJointPosition(int id, uint16_t &pos) override;
			bool setJointPosition(int id, uint16_t pos, uint16_t time) override;
			bool setManualModeAll(bool enable, int count) override;

		private:
			int fd_;
	};
}

#endif // ANGLEREADER_SERIAL__H
