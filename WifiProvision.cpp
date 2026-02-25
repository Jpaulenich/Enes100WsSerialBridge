// WifiProvision.cpp
#include "WifiProvision.h"
#include "Config.h"

static uint32_t s_lastWifiAttemptMs = 0;
static String s_lastPassword;

struct MacEntry {
  const char* hostname;
  const char* mac;
  const char* password;
};

// NOTE: Only entries with MAC+Password included.
// If any are missing, add them here.
static const MacEntry kTable[] PROGMEM = {
  {"KS-WM-001","98:CD:AC:0E:DA:C3","P6AWq4CzpQaQ"},
  {"KS-WM-002","50:02:91:74:B7:BB","YD4uGVJFPbRJ"},
  {"KS-WM-003","98:CD:AC:0E:5B:B8","6baPQdzDbHpP"},
  {"KS-WM-004","8C:AA:B5:C6:10:D0","44igm4mbcnHK"},
  {"KS-WM-005","C8:C9:A3:03:13:59","yFKkRq34eDsN"},
  {"KS-WM-006","18:FE:34:F5:FA:D8","EsCDRYWh4LUC"},
  {"KS-WM-007","DC:4F:22:7E:53:15","acCQ3rFMCaj9"},
  {"KS-WM-008","BC:FF:4D:37:D1:5C","i3udNcnXHaSb"},
  {"KS-WM-009","C8:C9:A3:02:27:6A","U7YkeCLfRxmj"},
  {"KS-WM-010","5C:CF:7F:83:C6:E4","qcH4N5UyfuzM"},
  {"KS-WM-011","BC:DD:C2:24:A8:6C","JTiKnCm4gs6D"},
  {"KS-WM-012","8C:AA:B5:15:EB:67","jT5iSrQquxpE"},
  {"KS-WM-013","98:CD:AC:0E:2D:8B","Ty9mMwxPrJPu"},
  {"KS-WM-014","EC:FA:BC:23:98:5B","J76eVgJHgFpW"},
  {"KS-WM-015","EC:FA:BC:23:9A:30","SHQ6T4ES4EwF"},
  {"KS-WM-016","C8:C9:A3:56:39:1F","feNe9xWNUSdc"},
  {"KS-WM-017","B4:E6:2D:67:FC:CF","jKzNrAsFn4KC"},
  {"KS-WM-018","48:3F:DA:64:13:14","kFMWm4msziqa"},
  {"KS-WM-019","48:3F:DA:69:0E:B0","nyNsMhhwVGkw"},
  {"KS-WM-020","8C:AA:B5:D4:F7:2D","KbuPjr4tEtEe"},
  {"KS-WM-021","50:02:91:74:08:DD","iRqU9XkieNFQ"},
  {"KS-WM-022","5C:CF:7F:01:7D:A8","M4DpSD4mJFzu"},
  {"KS-WM-023","84:F3:EB:83:58:94","XxRUWee9RwjT"},
  {"KS-WM-024","48:3F:DA:07:62:A5","YRf6ANPDCRUp"},
  {"KS-WM-025","68:C6:3A:F2:AF:70","EhiuiuYrA9un"},
  {"KS-WM-026","FC:F5:C4:97:BC:FA","LnzPHtvV3b64"},
  {"KS-WM-027","EC:FA:BC:13:31:9C","6sQgMNGtYU3e"},
  {"KS-WM-028","5C:CF:7F:E7:56:80","EEvdYtezjgwC"},
  {"KS-WM-029","80:7D:3A:42:54:2B","T6CAdc7UMUsH"},
  {"KS-WM-030","BC:DD:C2:24:A6:8D","pquxHRcF9i7z"},
  {"KS-WM-031","B4:E6:2D:67:FE:EE","G5qL7A4cQpQJ"},
  {"KS-WM-032","40:F5:20:F2:55:18","NR9uvSuaFifX"},
  {"KS-WM-033","DC:4F:22:6C:16:E0","M66gXsaht59C"},
  {"KS-WM-034","AC:0B:FB:CD:B0:67","hSGXmxEWiACT"},
  {"KS-WM-036","C8:C9:A3:56:60:CF","crcvNrz5emT9"},
  {"KS-WM-037","E0:98:06:8E:A3:FD","3N6KdQ6Ltymh"},
  {"KS-WM-038","68:C6:3A:A0:5D:34","7N9UHihxfnVR"},
  {"KS-WM-039","BC:FF:4D:EA:FF:EC","zXibWXfptYyr"},
  {"KS-WM-040","5C:CF:7F:18:F1:56","TzDyxp6KW4Xf"},
  {"KS-WM-041","84:0D:8E:8D:3E:51","kLrbhQkVkEnU"},
  {"KS-WM-042","C8:C9:A3:56:5F:B0","N4MjLLrxdF4M"},
  {"KS-WM-045","EC:FA:BC:23:89:86","7JDuMLthmuig"},
  {"KS-WM-046","C8:C9:A3:0A:AE:6C","aDcsXiCpRRDA"},
  {"KS-WM-047","3C:E9:0E:CB:62:87","eaphms4tTCJa"},
  {"KS-WM-048","68:C6:3A:DF:DA:EA","Yc3rvtaJ4sT5"},
  {"KS-WM-049","4C:EB:D6:E8:3E:A5","YhmSz9MnAeMA"},
  {"KS-WM-050","BC:DD:C2:24:A7:74","REjuLkjun3ut"},
  {"KS-WM-051","48:3F:DA:76:7D:93","F3HCHmUXeCqj"},
  {"KS-WM-052","48:E7:29:55:34:03","4tXXRHcyK6VG"},
  {"KS-WM-053","BC:FF:4D:EC:3F:E1","CsQAevDTKghm"},
  {"KS-WM-054","EC:FA:BC:23:8D:03","GDq3bRmuaygn"},
  {"KS-WM-055","EC:FA:BC:23:8E:18","nj6yvFhkpLV7"},
  {"KS-WM-056","FC:F5:C4:A9:00:4A","4A3JSbjyQ49r"},
  {"KS-WM-058","C8:C9:A3:46:57:C6","cieFEiuNaE3k"},
  {"KS-WM-059","F4:CF:A2:D2:4A:A9","Gjft5QvpgjgN"},
  {"KS-WM-061","84:F3:EB:4B:B6:BC","Ra6LbHsUsecQ"},
  {"KS-WM-062","E0:98:06:81:7C:61","QcvKkJPaUnfU"},
  {"KS-WM-063","8C:4F:00:42:34:F7","vGmuNEESjwDu"},
  {"KS-WM-064","BC:FF:4D:D8:7E:FB","Qbar5Y6Fsscd"},
  {"KS-WM-065","58:BF:25:D9:7E:1D","hLSsyAvmta6D"},
  {"KS-WM-068","C8:C9:A3:56:41:48","DVvKYwUyynH5"},
  {"KS-WM-069","C8:C9:A3:5D:98:A9","34ATA6ndFqcQ"},
  {"KS-WM-072","C8:C9:A3:A0:AA:7E","xpDs3fWkFv3h"},
  {"KS-WM-073","48:55:19:C0:E6:7C","9LFgAgrwRuqH"},
  {"KS-WM-074","8C:AA:B5:F4:9E:6B","ffvbE4Lg35Gd"},
  {"KS-WM-075","48:3F:DA:75:48:48","AgSzfrCfqYCC"},
  {"KS-WM-076","C4:5B:BE:DE:DC:78","vuWSvDPQfHUv"},
  {"KS-WM-077","98:CD:AC:0E:C4:F2","uv6H5jc6igaY"},
  {"KS-WM-079","48:3F:DA:88:15:AA","NHhHtrE6Cpce"},
  {"KS-WM-080","EC:FA:BC:23:98:60","uRA3r7a4ExWU"},
  {"KS-WM-081","D8:F1:5B:04:26:03","NfGQQHcCFycV"},
  {"KS-WM-082","CC:50:E3:D6:F6:2C","QuSUA4QzrgEx"},
  {"KS-WM-083","80:7D:3A:42:50:43","GgGx4VR6abjr"},
  {"KS-WM-084","98:CD:AC:0E:DB:FE","pWYmgyQK57MW"},
  {"KS-WM-085","50:02:91:74:98:1C","EJChjXnhdMK9"},
  {"KS-WM-086","08:3A:8D:E9:00:A9","JdacxL7z5e7A"},
  {"KS-WM-087","BC:DD:C2:BA:83:77","ehrYr3WCufVm"},
  {"KS-WM-089","4C:75:25:2D:D1:F4","Qs5JF9Cur75q"},
  {"KS-WM-090","DC:4F:22:0A:34:D1","9ku64EsRqkH5"},
  {"KS-WM-091","EC:FA:BC:23:97:6D","96DRudwMGQNV"},
  {"KS-WM-092","DC:4F:22:7D:C2:40","Cteqa3vEeMJj"},
  {"KS-WM-093","48:3F:DA:4C:87:9A","URre64JE4RJJ"},
  {"KS-WM-094","68:C6:3A:DF:D9:8A","hXuttSQMNKs4"},
  {"KS-WM-095","DC:4F:22:74:96:47","QLt96xvVQ5M6"},
  {"KS-WM-096","D8:F1:5B:04:73:74","jmeCktDyDkuu"},
  {"KS-WM-097","C8:C9:A3:56:3E:27","MFqiim5wdLDa"},
  {"KS-WM-098","DC:4F:22:7D:C2:3A","ATvL9Dr7GF9i"},
  {"KS-WM-099","98:CD:AC:0E:64:98","4FeUvWuwSbqd"},
  {"KS-WM-102","2C:3A:E8:26:79:A8","9RV7juu6iprv"},
  {"KS-WM-103","FC:F5:C4:A7:91:BD","CDPutc4Rk7Lj"},
  {"KS-WM-105","C8:C9:A3:45:02:F9","6CVEsG5sMKWx"},
  {"KS-WM-106","80:7D:3A:02:8B:AB","GEg9sUmsyMdN"},
  {"KS-WM-107","84:F3:EB:04:3E:12","i9MvuWY9ckYF"},
  {"KS-WM-108","84:F3:EB:E0:B9:6C","yMGNiz7JJeTr"},
  {"KS-WM-109","CC:50:E3:53:34:13","Aa5MXuzcGeqT"},
  {"KS-WM-110","B4:E6:2D:75:86:5A","wX7kzAxjALcG"},
  {"KS-WM-111","BC:DD:C2:02:38:8B","w7YtavgtmskT"},
  {"KS-WM-112","EC:FA:BC:6C:17:04","eqY4myc4AiqW"},
  {"KS-WM-113","F4:CF:A2:55:A3:6F","vrCxNgvewcz4"},
  {"KS-WM-114","4C:11:AE:13:BB:E1","x6J7Fsfjzwuj"},
  {"KS-WM-115","EC:FA:BC:B9:E0:3B","ScxQR6PAQMwL"},
  {"KS-WM-116","80:7D:3A:7B:F6:54","TvyApFbxGsdh"},
  {"KS-WM-117","E8:DB:84:BC:F6:CF","pxibjTDVRhyj"},
  {"KS-WM-118","F4:CF:A2:64:22:C7","GYUKnmynthXz"},
  {"KS-WM-119","8C:AA:B5:F9:65:82","r3YNK3Tt7xYD"},
  {"KS-WM-120","2C:F4:32:1E:36:13","eTfCqPvzUWh4"},
  {"KS-WM-121","B4:E6:2D:75:84:A2","uKv3KFFuv774"},
  {"KS-WM-122","2C:F4:32:53:C8:BF","Qm3xSDYm4t7W"},
  {"KS-WM-123","CC:50:E3:2E:18:76","beQEPGg3PxWv"},
  {"KS-WM-124","84:F3:EB:9A:18:D5","QQ5UnVH9b7KV"},
};

String WifiProvision::getMacString() {
  WiFi.mode(WIFI_STA);
  delay(10);
  String mac = WiFi.macAddress();
  mac.toUpperCase();
  return mac;
}

bool WifiProvision::macEqualsIgnoreCase(const String& a, const String& b) {
  if (a.length() != b.length()) return false;
  for (size_t i = 0; i < a.length(); i++) {
    char ca = a[i];
    char cb = b[i];
    if (ca >= 'a' && ca <= 'z') ca = (char)(ca - 'a' + 'A');
    if (cb >= 'a' && cb <= 'z') cb = (char)(cb - 'a' + 'A');
    if (ca != cb) return false;
  }
  return true;
}

bool WifiProvision::getCredentialsForThisDevice(Credentials& out) {
  const String myMac = getMacString();

  for (size_t i = 0; i < (sizeof(kTable) / sizeof(kTable[0])); i++) {
    MacEntry entry;
    memcpy_P(&entry, &kTable[i], sizeof(MacEntry));

    String mac = String(entry.mac);
    mac.toUpperCase();

    if (macEqualsIgnoreCase(myMac, mac)) {
      const String pwd = String(entry.password ? entry.password : "");
      if (pwd.length() == 0) return false;

      out.hostname = String(entry.hostname);
      out.mac = mac;
      out.password = pwd;
      return true;
    }
  }
  return false;
}

void WifiProvision::applyHostname(const String& hostname) {
  WiFi.hostname(hostname);
}

bool WifiProvision::connect(const char* ssid, const String& password) {
  WiFi.mode(WIFI_STA);
  delay(10);

  s_lastPassword = password;
  WiFi.begin(ssid, password.c_str());

  const uint32_t start = millis();
  while (!WiFi.isConnected() && (millis() - start) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    yield();
  }
  return WiFi.isConnected();
}

void WifiProvision::ensureConnected(const char* ssid) {
  if (WiFi.isConnected()) return;

  const uint32_t now = millis();
  if (now - s_lastWifiAttemptMs < WIFI_RETRY_BACKOFF_MS) return;
  s_lastWifiAttemptMs = now;

  if (s_lastPassword.length() == 0) return;

  WiFi.disconnect();
  delay(50);
  WiFi.begin(ssid, s_lastPassword.c_str());
}