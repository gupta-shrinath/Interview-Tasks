import 'dart:convert';

import 'package:contacts/main.dart';
import 'package:contacts/models/contact.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:http/http.dart';
import 'package:http/testing.dart';

void main() {
  group('fetchContacts', () {
    test('returns list of Contact if http call completes', () async {
      final client = MockClient((request) async {
        final results = [
          {
            "_id": "157550910dc34bc588d6d5b449c53be5",
            "contact_number": "8433530825",
            "first_name": "Shrinath",
            "last_name": "Gupta"
          },
          {
            "_id": "78c54dee47544f7d8c667ae438f38116",
            "contact_number": "7977563530",
            "first_name": "Dilesh",
            "last_name": "Tanna"
          },
          {
            "_id": "acf4ea6c35ca47399c15bf551afe6db2",
            "contact_number": "7977563530",
            "first_name": "dilesh",
            "last_name": "tanna"
          },
          {
            "_id": "caed5b05fd2748e7a25673fcdd08219c",
            "contact_number": "2255336644",
            "first_name": "hsh",
            "last_name": "bsb"
          },
          {
            "_id": "b45867188fc3442985d54d233c7fdefb",
            "contact_number": "2255883366",
            "first_name": "gg",
            "last_name": "cf"
          }
        ];
        return Response(jsonEncode(results), 200);
      });
      expect(await fetchContacts(client), isA<List<Contact>>());
    });
    test('throws an exception if http call returns empty list', () async {
      final client = MockClient((request) async {
        return Response(jsonEncode({}), 200);
      });
      expect(
          await fetchContacts(client), throwsA(const TypeMatcher<Exception>()));
    });
  });
}
