#include <stdio.h>

NtripClient::NtripClient(std::string host, int port, 
            std::string mount, std::string user, 
            std::string pw)
  : host_(host),
    port_(port),
    mount_(mount),
    user_(user),
    pw_(pw)
{
}

NtripClient::~NtripClient() = default;

bool NtripClient::start() 
{
    rtcm = std::thread(&NtripClient::sendRtcm, this);
}

bool NtripClient::stop()
{
    rtcm.join();
}

static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
  auto cnt = ((boost::asio::serial_port*)userp)->write_some(boost::asio::buffer((char*)contents, size*nmemb));
  return cnt;
}

void NtripClient::sendRtcm() {
  CURL *curl;
  CURLcode res;
  std::string readBuffer;

  curl = curl_easy_init();
  if(!curl) {
      return;
  }

  curl_easy_setopt(curl, CURLOPT_URL, connString());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
  curl_easy_setopt(curl, CURLOPT_USERPWD, authString());
  curl_easy_setopt(curl, CURLOPT_USERAGENT, "StreetSmarts/1.0");

  res = curl_easy_perform(curl);

  if(res != CURLE_OK) 
    fprintf(stderr, "curl_easy_perform() failed: %s\n",
            curl_easy_strerror(res));

  curl_easy_cleanup(curl);
}

std::string connString(){
  std::ostringstream cstring;
  cstring << "http://" << host_ << ":" << port_
          << "/" << mount_;
  return cstring.str();
}

std::string authString(){
    std::ostringstream astring;
    astring  << user_ << ":" << pw_;
    return astring.str();
}
