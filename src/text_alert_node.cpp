/*
* This example demonstrates sending a SMS or MMS in C++ using the Twilio REST
* APIs.  After building, you should be able to run it with:
* 
* ./bin/cpp_demo
* 
*/

#include <iostream>
#include <string>
#include <memory>
#include <cstdlib>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "twilio.hh"

#include <std_srvs/Empty.h>

int main(int argc, char * argv[])
{
        std::string account_sid = std::getenv("TWILIO_ACCOUNT_SID");
        std::string auth_token = std::getenv("TWILIO_AUTH_TOKEN");
        std::string message = "Testing Env Var";
        std::string from_number = "+16473720640";
        std::string to_number = "+16479663926";

        // Instantiate a twilio object and call send_message
        std::string response;
        auto twilio = std::make_shared<twilio::Twilio>(
            account_sid, 
            auth_token
        );
        
        bool message_success = twilio->send_message(
                to_number, 
                from_number, 
                message
        );

        // Report success or failure
        if (!message_success) {
                std::cout << "Message send failed." << std::endl;
                return -1;
	}
        else 
	{
                std::cout << "SMS sent successfully!" << std::endl;
        }

        return 0;
}
