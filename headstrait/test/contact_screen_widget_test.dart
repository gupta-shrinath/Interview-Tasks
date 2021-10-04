import 'dart:async';

import 'package:contacts/main.dart';
import 'package:contacts/models/contact.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets("Show circular progress indicator when future isn't complete",
      (WidgetTester tester) async {
    tester.runAsync(() async {
      final contactsScreen = MaterialApp(
        home: Scaffold(
          body: buildContactList(
            contacts: Future.delayed(Duration(milliseconds: 5)),
          ),
        ),
      );
      await tester.pumpWidget(contactsScreen);
      expect(find.byType(CircularProgressIndicator), findsOneWidget);
    });
  });

  testWidgets("Show error text widget when future returns error",
      (WidgetTester tester) async {
    final contactsScreen = MaterialApp(
      home: Scaffold(
        body: buildContactList(
          contacts: Future.delayed(Duration(milliseconds: 10), () {
            throw Exception();
          }),
        ),
      ),
    );
    await tester.pumpWidget(contactsScreen);
    await tester.pumpAndSettle();
    final errorContactText = find.text("Oops contact load failed !");
    expect(errorContactText, findsOneWidget);
  });

  testWidgets("Show contact in ListTile when future returns contact list",
      (WidgetTester tester) async {
    final contactsScreen = MaterialApp(
      home: buildContactList(
        contacts: Future.value(<Contact>[
          Contact(
              firstName: "Shrinath",
              lastName: "Gupta",
              contactNumber: "contactNumber")
        ]),
      ),
    );
    await tester.pumpWidget(contactsScreen);
    await tester.pumpAndSettle();
    final errorContactText = find.widgetWithText(ListTile, 'Shrinath Gupta');
    expect(errorContactText, findsOneWidget);
  });
}
