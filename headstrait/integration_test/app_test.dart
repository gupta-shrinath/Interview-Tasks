import 'package:contacts/main.dart' as app;
import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';

void main() {
  IntegrationTestWidgetsFlutterBinding.ensureInitialized();
  group('end-to-end test', () {
    testWidgets('add contact to list', (WidgetTester tester) async {
      app.main();
      await tester.pumpAndSettle();
      final Finder fab = find.byTooltip('Add');
      await tester.tap(fab);
      await tester.pumpAndSettle();
      Finder firstName = find.byKey(Key('firstName'));
      await tester.tap(firstName);
      await tester.enterText(firstName, 'John');
      Finder lastName = find.byKey(Key('lastName'));
      await tester.tap(lastName);
      await tester.enterText(lastName, 'Doe');
      Finder contactNumber = find.byKey(Key('contactNumber'));
      await tester.tap(contactNumber);
      await tester.enterText(contactNumber, "1122334455");
      Finder button = find.byKey(Key('submit'));
      await tester.press(button);
      await tester.pumpAndSettle();
      expect(find.text('John'), findsOneWidget);
    });
  });
}
