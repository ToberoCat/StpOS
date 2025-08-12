enum WifiBand {
  band2_4GHz,
  band5GHz,
  bandAuto,
}

extension WifiBandExtension on WifiBand {
  String get displayName {
    switch (this) {
      case WifiBand.band2_4GHz:
        return '2.4 GHz';
      case WifiBand.band5GHz:
        return '5 GHz';
      case WifiBand.bandAuto:
        return 'Auto';
    }
  }

  String get nmcliValue {
    switch (this) {
      case WifiBand.band2_4GHz:
        return 'bg';
      case WifiBand.band5GHz:
        return 'a';
      case WifiBand.bandAuto:
        return 'a';
    }
  }

  List<int> get channels {
    switch (this) {
      case WifiBand.band2_4GHz:
        return [1, 6, 11];       case WifiBand.band5GHz:
        return [36, 40, 44, 48, 149, 153, 157, 161];       case WifiBand.bandAuto:
        return [36, 40, 44, 48, 149, 153, 157, 161];     }
  }
}