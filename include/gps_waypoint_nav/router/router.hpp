#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <iomanip>


#include <nlohmann/json.hpp>
#include <curl/curl.h>

class Router {
public:
    Router(){}
    Router(const std::string&api_key, const std::vector<double>&start, const std::vector<double>&end);
    void viewRoute() const;
    std::vector<std::vector<double>> getCoordinates();

private:
    std::string api_key_;
    std::vector<double> start_;
    std::vector<double> end_;
    std::vector<std::vector<double>> coordinates_;

    static size_t write_callback(char* ptr, size_t size, size_t nmemb, void* userdata);
    std::string send_http_request() const;
};

Router::Router(const std::string&api_key, const std::vector<double>&start, const std::vector<double>&end)
: api_key_(api_key), start_(start), end_(end)
{
    if (start.size() != 2 || end.size() != 2) {
        throw std::invalid_argument("start와 end 벡터의 크기가 유효하지 않습니다.");
    }

    nlohmann::json j = nlohmann::json::parse(send_http_request());

    auto features = j["features"];
    auto geometry = features[0]["geometry"];
    coordinates_ = geometry["coordinates"];
}

void Router::viewRoute() const{
    std::cout << "경로 정보" << std::endl;
    for (std::vector<double> coord : coordinates_) {
        std::cout << "경도: " << coord[0] << ", 위도: " << coord[1] << std::endl; 
    }
}

std::vector<std::vector<double>> Router::getCoordinates(){
    return coordinates_;
}

size_t Router::write_callback(char* ptr, size_t size, size_t nmemb, void* userdata) {
    ((std::string*)userdata)->append(ptr, size * nmemb);
    return size * nmemb;
}

std::string Router::send_http_request() const{
    // HTTP 요청 작성
    std::string url = "https://api.openrouteservice.org/v2/directions/driving-car?api_key=" + api_key_;
    url += "&start=" + std::to_string(start_[0]) + "," + std::to_string(start_[1]);
    url += "&end=" + std::to_string(end_[0]) + "," + std::to_string(end_[1]);
    url += "&geometry_format=geojson";

    //std::cout << url << "\n";

    CURL *curl = curl_easy_init();
    std::string response;

    if (curl){
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &Router::write_callback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

        CURLcode result = curl_easy_perform(curl);
        if (result != CURLE_OK)
            std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(result) << std::endl;

        curl_easy_cleanup(curl);
    }

    return response;
}