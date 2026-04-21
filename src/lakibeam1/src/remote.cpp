#include "lakibeam1/remote.hpp"

#include <curl/curl.h>

#include <mutex>
#include <string>

namespace
{

std::once_flag g_curl_init_once;

void init_curl()
{
  curl_global_init(CURL_GLOBAL_DEFAULT);
}

size_t discard_response(void *, size_t size, size_t nmemb, void *)
{
  return size * nmemb;
}

}  // namespace

bool sensor_config(const std::string & sensor_ipaddr, const std::string & parameter, const std::string & value)
{
  std::call_once(g_curl_init_once, init_curl);

  CURL * curl = curl_easy_init();
  if (curl == nullptr) {
    return false;
  }

  const std::string url = "http://" + sensor_ipaddr + parameter;
  long http_code = 0;

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, value.c_str());
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 3L);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, discard_response);
  curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 1L);

  const CURLcode result = curl_easy_perform(curl);
  if (result == CURLE_OK) {
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
  }

  curl_easy_cleanup(curl);
  return result == CURLE_OK && http_code == 200;
}
