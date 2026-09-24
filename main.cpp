#include <print>
#include <string>
#include <vector>
#include <ranges>
#include <optional>
#include <cmath>
#include <charconv>

constexpr std::int64_t NAVIGATION_STATUS_SENTINEL_VALUE = 15;
constexpr std::int64_t LONGITUDE_SENTINEL_VALUE = 108'600'000;
constexpr std::int64_t LATITUDE_SENTINEL_VALUE = 54'600'000;
constexpr std::int64_t RATE_OF_TURN_SENTINEL_VALUE = -128;
constexpr std::int64_t SPEED_OVER_GROUND_SENTINEL_VALUE = 1023;
constexpr std::int64_t COURSE_OVER_GROUND_SENTINEL_VALUE = 3600;
constexpr std::int64_t TRUE_HEADING_SENTINEL_VALUE = 511;

/*
┌─────────┬─────┬────────────────┬──────┬─────────────────────────────────────────┐
│  bits   │  w  │     field      │ sign │            units / sentinel             │
├─────────┼─────┼────────────────┼──────┼─────────────────────────────────────────┤
│ 0–5     │ 6   │ message type   │ U    │                                         │
├─────────┼─────┼────────────────┼──────┼─────────────────────────────────────────┤
│ 6–7     │ 2   │ repeat         │ U    │                                         │
│         │     │ indicator      │      │                                         │
├─────────┼─────┼────────────────┼──────┼─────────────────────────────────────────┤
│ 8–37    │ 30  │ MMSI           │ U    │                                         │
├─────────┼─────┼────────────────┼──────┼─────────────────────────────────────────┤
│ 38–41   │ 4   │ navigation     │ U    │ 15 = undefined                          │
│         │     │ status         │      │                                         │
├─────────┼─────┼────────────────┼──────┼─────────────────────────────────────────┤
│ 42–49   │ 8   │ rate of turn   │ S    │ −128 = not available                    │
├─────────┼─────┼────────────────┼──────┼─────────────────────────────────────────┤
│ 50–59   │ 10  │ speed over     │ U    │ 1/10 knot; 1023 = N/A, 1022 = ≥102.2 kn │
│         │     │ ground         │      │                                         │
├─────────┼─────┼────────────────┼──────┼─────────────────────────────────────────┤
│ 60      │ 1   │ position       │ U    │ 1 = DGPS (<10 m), 0 = GNSS              │
│         │     │ accuracy       │      │                                         │
├─────────┼─────┼────────────────┼──────┼─────────────────────────────────────────┤
│ 61–88   │ 28  │ longitude      │ S    │ 1/10000 min; 108600000 = N/A            │
├─────────┼─────┼────────────────┼──────┼─────────────────────────────────────────┤
│ 89–115  │ 27  │ latitude       │ S    │ 1/10000 min; 54600000 = N/A             │
├─────────┼─────┼────────────────┼──────┼─────────────────────────────────────────┤
│ 116–127 │ 12  │ course over    │ U    │ 1/10 degree; 3600 = N/A                 │
│         │     │ ground         │      │                                         │
├─────────┼─────┼────────────────┼──────┼─────────────────────────────────────────┤
│ 128–136 │ 9   │ true heading   │ U    │ degrees; 511 = N/A                      │
├─────────┼─────┼────────────────┼──────┼─────────────────────────────────────────┤
│ 137–142 │ 6   │ timestamp      │ U    │ UTC second; 60 = N/A, 61 = manual, 62 = │
│         │     │                │      │  dead reckoning, 63 = inoperative       │
├─────────┼─────┼────────────────┼──────┼─────────────────────────────────────────┤
│ 143–144 │ 2   │ manoeuvre      │ U    │ 0 = N/A, 1 = none, 2 = special          │
│         │     │ indicator      │      │                                         │
├─────────┼─────┼────────────────┼──────┼─────────────────────────────────────────┤
│ 145–147 │ 3   │ spare          │ U    │                                         │
├─────────┼─────┼────────────────┼──────┼─────────────────────────────────────────┤
│ 148     │ 1   │ RAIM flag      │ U    │                                         │
├─────────┼─────┼────────────────┼──────┼─────────────────────────────────────────┤
│ 149–167 │ 19  │ radio status   │ U    │ SOTDMA state                            │
└─────────┴─────┴────────────────┴──────┴─────────────────────────────────────────┘
*/

struct Message{
    int64_t type;
    int64_t repeat_indicator;
    int64_t MMSI;
    std::optional<int64_t> navigation_status;
    std::optional<double> rate_of_turn;
    std::optional<double> speed_over_ground;
    int64_t position_accuracy;
    std::optional<double> longitude;
    std::optional<double> latitude;
    std::optional<double> course_over_ground;
    std::optional<int64_t> true_heading;
    int64_t timestamp;
    int64_t manoeuvre_indicator;
    int64_t spare;
    int64_t RAIM_flag;
    int64_t radio_status;
};

enum Sign{
    Signed = true,
    Unsigned = false
};

void decode_char(char& val){
    if(val > 87+8)
        val -= 8;
    val-=48;
}

std::int64_t convert_from_bits(std::uint32_t start, std::uint32_t bits_length, std::string_view val, Sign s){
    std::uint32_t start_bit = start % 6;
    std::uint32_t current_index = (start - start_bit)/6;

    if(current_index >= val.length())
        return 0;
    char current_char = val[current_index];
    decode_char(current_char);

    std::uint32_t count = start_bit;
    std::int64_t total = 0;

    std::uint32_t length = bits_length;
    while(length > 0){
        if(count == 6){
            current_index++;
            if(current_index >= val.length())
                break;
            current_char = val[current_index];
            decode_char(current_char);
            count = 0;
        }
        total <<= 1;
        total |= (current_char >> (5 - count)) & 1;

        count++;
        length--;
    }
    if(s == Signed){
        if(((total >> (bits_length - 1)) & 1) == 1){
            total |= ((1LL << (64 - bits_length)) - 1) << bits_length;
        }
    }

    return total;
}

std::optional<Message> convert_to_message(std::string_view sentence){
        auto parts = sentence | std::views::split(',');
        auto vec = parts | std::views::transform([](auto p){return std::string_view(p);}) | std::ranges::to<std::vector>();
        if(vec.size() != 7)
            return std::nullopt;
        std::string_view payload = vec[5];

        std::int64_t message_type = convert_from_bits(0, 6, payload, Unsigned);
        std::int64_t repeat_indicator = convert_from_bits(6, 2, payload, Unsigned);
        std::int64_t mmsi = convert_from_bits(8, 30, payload, Unsigned);
        std::int64_t navigation_status_raw = convert_from_bits(38, 4, payload, Unsigned);
        std::int64_t rate_of_turn_raw = convert_from_bits(42, 8, payload, Signed);
        std::int64_t speed_over_ground_raw = convert_from_bits(50, 10, payload, Unsigned);
        std::int64_t position_accuracy = convert_from_bits(60, 1, payload, Unsigned);

        std::int64_t longitude_raw = convert_from_bits(61, 28, payload, Signed);
        std::int64_t latitude_raw = convert_from_bits(89, 27, payload, Signed);
        std::int64_t course_over_ground_raw = convert_from_bits(116, 12, payload, Unsigned);
        std::int64_t true_heading_raw = convert_from_bits(128, 9, payload, Unsigned);
        std::int64_t timestamp = convert_from_bits(137, 6, payload, Unsigned);
        std::int64_t manoeuvre_indicator = convert_from_bits(143, 2, payload, Unsigned);
        std::int64_t spare = convert_from_bits(145, 3, payload, Unsigned);
        std::int64_t raim_flag = convert_from_bits(148, 1, payload, Unsigned);
        std::int64_t radio_status = convert_from_bits(149, 19, payload, Unsigned);

        std::optional<int64_t> navigation_status = navigation_status_raw == NAVIGATION_STATUS_SENTINEL_VALUE ? std::nullopt : std::optional<int64_t>(navigation_status_raw);

        std::optional<double> rate_of_turn = rate_of_turn_raw == RATE_OF_TURN_SENTINEL_VALUE ? std::nullopt : std::optional<double>(std::copysign(std::pow((static_cast<double>(rate_of_turn_raw) / 4.733), 2), rate_of_turn_raw));

        std::optional<double> speed_over_ground = speed_over_ground_raw == SPEED_OVER_GROUND_SENTINEL_VALUE ? std::nullopt : std::optional<double>(static_cast<double>(speed_over_ground_raw)/10.0);

        std::optional<double> longitude = longitude_raw == LONGITUDE_SENTINEL_VALUE ? std::nullopt : std::optional<double>(static_cast<double>(longitude_raw)/600000.0);

        std::optional<double> latitude = latitude_raw == LATITUDE_SENTINEL_VALUE ? std::nullopt : std::optional<double>(static_cast<double>(latitude_raw)/600000.0);

        std::optional<double> course_over_ground = course_over_ground_raw == COURSE_OVER_GROUND_SENTINEL_VALUE ? std::nullopt : std::optional<double>(static_cast<double>(course_over_ground_raw)/10.0);
        std::optional<int64_t> true_heading = true_heading_raw == TRUE_HEADING_SENTINEL_VALUE ? std::nullopt : std::optional<int64_t>(true_heading_raw);

        Message message = Message{
            .type = message_type,
            .repeat_indicator = repeat_indicator,
            .MMSI = mmsi,
            .navigation_status = navigation_status,
            .rate_of_turn = rate_of_turn,
            .speed_over_ground = speed_over_ground,
            .position_accuracy = position_accuracy,
            .longitude = longitude,
            .latitude = latitude,
            .course_over_ground = course_over_ground,
            .true_heading = true_heading,
            .timestamp = timestamp,
            .manoeuvre_indicator = manoeuvre_indicator,
            .spare = spare,
            .RAIM_flag = raim_flag,
            .radio_status = radio_status
        };

        return message;
}

void print_message(Message const &m){
    std::println("----------------------------------------");
    std::println("message_type: {:d}", m.type);
    std::println("repeat_indicator: {:d}", m.repeat_indicator);
    std::println("MMSI: {:d}", m.MMSI);

    if(m.navigation_status.has_value())
        std::println("navigation_status: {:d}", *m.navigation_status);
    else
        std::println("navigation_status: NO VALUE");

    if(m.rate_of_turn.has_value())
        std::println("rate_of_turn: {:f}", *m.rate_of_turn);
    else
        std::println("rate_of_turn: NO VALUE");

    if(m.speed_over_ground.has_value())
        std::println("speed_over_ground: {:f}", *m.speed_over_ground);
    else
        std::println("speed_over_ground: NO VALUE");

    std::println("position_accuracy: {:d}", m.position_accuracy);

    if(m.longitude.has_value())
        std::println("longitude: {:f}", *m.longitude);
    else
        std::println("longitude: NO VALUE");

    if(m.latitude.has_value())
        std::println("latitude: {:f}", *m.latitude);
    else
        std::println("latitude: NO VALUE");

    if(m.course_over_ground.has_value())
        std::println("course_over_ground: {:f}", *m.course_over_ground);
    else
        std::println("course_over_ground: NO VALUE");

    if(m.true_heading.has_value())
        std::println("true_heading: {:d}", *m.true_heading);
    else
        std::println("true_heading: NO VALUE");

    switch(m.timestamp){
        case 60:
            std::println("timestamp: NOT AVAILABLE");
            break;
        case 61:
            std::println("timestamp: MANUAL INPUT");
            break;
        case 62:
            std::println("timestamp: DEAD RECKONING");
            break;
        case 63:
            std::println("timestamp: INOPERATIVE");
            break;
        default:
            std::println("timestamp: {:d}", m.timestamp);
    }

    std::println("manoeuvre_indicator: {:d}", m.manoeuvre_indicator);
    std::println("spare: {:d}", m.spare);
    std::println("RAIM_flag: {:d}", m.RAIM_flag);
    std::println("radio_status: {:d}", m.radio_status);
    std::println("----------------------------------------");
}

bool validate_checksum(std::string_view sentence){
    if(sentence.find("!") == std::string_view::npos || sentence.find("*") == std::string_view::npos || sentence.find("!") > sentence.find("*")){
        std::println("Incorrect sentence: {}", sentence);
        return false;
    }

    auto substring = sentence.substr(sentence.find("!")+1, sentence.find("*") - (sentence.find("!")+1));
    auto valid_checksum_substring = sentence.substr(sentence.find("*") + 1, sentence.length()-1);
    std::uint8_t valid_checksum;

    auto [_, ec]= std::from_chars(valid_checksum_substring.data(), valid_checksum_substring.data() + valid_checksum_substring.length(), valid_checksum, 16);
    if(ec != std::errc{}){
        std::println("Invalid checksum: {}", valid_checksum_substring);
        return false;
    }

    std::uint8_t checksum = 0x00;
    for(auto &c : substring)
        checksum ^= c;
    std::println("Checksum: {:x}", checksum);
    std::println("Valid_checksum: {:x}", valid_checksum);

    if(valid_checksum == checksum)
        return true;
    return false;
}

int main(){
    //San Francisco
    std::println("San Francisco");
    std::string sentence_san_francisco = "!AIVDM,1,1,,A,15M67FC000G?ufbE`FepT@3n00Sa,0*5C";

    if(!validate_checksum(sentence_san_francisco)){
        std::println("CHECKSUM MISMATCH!");
    }
    else{
        std::optional<Message> san_francisco_message = convert_to_message(sentence_san_francisco);
        if(san_francisco_message.has_value()){
            print_message(*san_francisco_message);
        }
        else{
            std::println("INVALID MESSAGE");
        }
    }

    //Sydney
    std::println("Sydney");
    std::string sentence_sydney = "!AIVDM,1,1,,A,17Ol>00000:l;VIdWd000?v00000,0*61";

    if(!validate_checksum(sentence_sydney)){
        std::println("CHECKSUM MISMATCH!");
    }
    else{
        std::optional<Message> sydney_message = convert_to_message(sentence_sydney);
        if(sydney_message.has_value()){
            print_message(*sydney_message);
        }
        else{
            std::println("INVALID MESSAGE");
        }
    }

    //Corrupt San Francisco
    std::println("Corrupt San Francisco");
    std::string sentence_corrupt_san_francisco = "!AIVDM,1,1,,A,15M67FOP?w<tSF0l4Q@>4?wp0000,0*69";

    if(!validate_checksum(sentence_corrupt_san_francisco)){
        std::println("CHECKSUM MISMATCH!");
    }
    else{
        std::optional<Message> corrupt_san_francisco_message = convert_to_message(sentence_corrupt_san_francisco);
        if(corrupt_san_francisco_message.has_value()){
            print_message(*corrupt_san_francisco_message);
        }
        else{
            std::println("INVALID MESSAGE");
        }
    }

    //Invalid message
    std::println("Invalid message");
    std::string sentence_invalid_message = "hello";

    if(!validate_checksum(sentence_invalid_message)){
        std::println("CHECKSUM MISMATCH!");
    }
    else{
        std::optional<Message> invalid_message_message = convert_to_message(sentence_invalid_message);
        if(invalid_message_message.has_value()){
            print_message(*invalid_message_message);
        }
        else{
            std::println("INVALID MESSAGE");
        }
    }

    std::string sentence_frag_1_ch_1= "!AIVDM,2,1,3,B,53m=TNP2;=`0h77;?@1@E=B1HE=<Dh0000000016<PD886J>N@E4SkDkp5R@,0*49";
    std::string sentence_frag_2_ch_1 = "!AIVDM,2,2,3,B,H0Si3h00000,2*3D";

    std::string sentence_frag_1_ch_2 = "!AIVDM,2,1,7,A,53otHiP2;=`0h77;?@1<D<tpB1<PU00000000016<PD886J>N@@QDQiCP000,0*10";
    std::string sentence_frag_2_ch_2 = "!AIVDM,2,2,7,A,00000000000,2*23";
}
