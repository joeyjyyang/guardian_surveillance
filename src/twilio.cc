#include <sstream>
#include <curl/curl.h>
#include <locale>
#include <codecvt>
#include <string>

#include "twilio.hh"

namespace twilio {

// Portably ignore curl response
size_t Twilio::_null_write(
        char *ptr, 
        size_t size, 
        size_t nmemb, 
        void *userdata)
{
        return size*nmemb;
}

// Write curl response to a stringstream
size_t Twilio::_stream_write(
        char *ptr,
        size_t size,
        size_t nmemb,
        void *userdata) 
{
        size_t response_size = size * nmemb;
        std::stringstream *ss = (std::stringstream*)userdata;
        ss->write(ptr, response_size);
        return response_size;
}

bool Twilio::send_message(
        std::string const& to_number,
        std::string const& from_number,
        std::string const& message_body
        )
{
        std::stringstream response_stream;
        std::u16string converted_message_body;

	std::wstring_convert<std::codecvt_utf8<char16_t>, char16_t> convert;
	converted_message_body = convert.from_bytes(message_body);

        CURL *curl;
        curl_global_init(CURL_GLOBAL_ALL);
        curl = curl_easy_init();

        // Percent encode special characters
        char *message_body_escaped = curl_easy_escape(
                curl, 
                message_body.c_str(), 
                0
        );

        std::stringstream url;
        std::string url_string;
        url << "https://api.twilio.com/2010-04-01/Accounts/" << account_sid
                << "/Messages";
        url_string = url.str();

        std::stringstream parameters;
        std::string parameter_string;
        parameters << "To=" << to_number << "&From=" << from_number 
                << "&Body=" << message_body_escaped;
        parameter_string = parameters.str();

        curl_easy_setopt(curl, CURLOPT_POST, 1);
        curl_easy_setopt(curl, CURLOPT_URL, url_string.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, parameter_string.c_str());
        curl_easy_setopt(curl, CURLOPT_USERNAME, account_sid.c_str());
        curl_easy_setopt(curl, CURLOPT_PASSWORD, auth_token.c_str());
        
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, _null_write);

	CURLcode res = curl_easy_perform(curl);     
        
	curl_free(message_body_escaped);
        curl_easy_cleanup(curl);
        long http_code = 0;
        curl_easy_getinfo (curl, CURLINFO_RESPONSE_CODE, &http_code);

	return true;
}

} // end namespace twilio
