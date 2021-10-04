import 'dart:async';
import 'dart:convert';

import 'package:contacts/models/contact.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:http/http.dart' as http;

void main() {
  runApp(App());
}

class App extends StatelessWidget {
  const App({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      title: 'Contacts',
      home: ContactsScreen(),
    );
  }
}

class ContactsScreen extends StatefulWidget {
  const ContactsScreen({Key? key}) : super(key: key);

  @override
  _ContactsScreenState createState() => _ContactsScreenState();
}

class _ContactsScreenState extends State<ContactsScreen> {
  final _formKey = GlobalKey<FormState>();
  TextEditingController firstNameTextController = TextEditingController();
  TextEditingController lastNameTextController = TextEditingController();
  TextEditingController contactNumberTextController = TextEditingController();
  late Future contacts;

  @override
  void initState() {
    super.initState();
    contacts = fetchContacts(http.Client());
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text("Contacts"),
      ),
      body: RefreshIndicator(
        onRefresh: () {
          setState(() {
            contacts = fetchContacts(http.Client());
          });
          return contacts;
        },
        child: buildContactList(contacts: contacts),
      ),
      floatingActionButton: FloatingActionButton(
        tooltip: "Add",
        onPressed: () {
          showModalBottomSheet(
              context: context,
              isScrollControlled: true,
              builder: (context) => buildAddContactSheet());
        },
        child: Icon(Icons.add),
      ),
    );
  }

  @override
  void dispose() {
    // firstNameTextController.dispose();
    // lastNameTextController.dispose();
    // contactNumberTextController.dispose();
    super.dispose();
  }

  void addContact() async {
    final body = jsonEncode({
      'first_name': firstNameTextController.text,
      'last_name': lastNameTextController.text,
      'contact_number': contactNumberTextController.text
    });
    final header = <String, String>{
      'Content-Type': 'application/json; charset=UTF-8',
    };

    final response =
        await http.post(Uri.parse(url), headers: header, body: body);
    if (response.statusCode != 201) {
      ScaffoldMessenger.of(context)
          .showSnackBar(SnackBar(content: Text("Contact add failed")));
    } else if (response.statusCode == 201) {
      print("Add success");
      setState(() {
        contacts = fetchContacts(http.Client());
      });
    }
  }

  void clearTexts() {
    firstNameTextController.clear();
    lastNameTextController.clear();
    contactNumberTextController.clear();
  }

  Widget buildAddContactSheet() {
    final firstNameKey = Key("firstName");
    final lastNameKey = Key("lastName");
    final contactNumberKey = Key("contactNumber");
    final submitKey = Key("submit");

    return Padding(
      padding: MediaQuery.of(context).viewInsets,
      child: Container(
        padding: const EdgeInsets.all(20),
        child: Form(
          key: _formKey,
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              TextFormField(
                key: firstNameKey,
                controller: firstNameTextController,
                keyboardType: TextInputType.name,
                decoration: InputDecoration(
                    hintText: "First Name",
                    labelText: "First Name",
                    border: OutlineInputBorder()),
                validator: (value) {
                  if (value == null || value.isEmpty) {
                    return 'Please enter first name';
                  }
                  return null;
                },
              ),
              SizedBox(
                height: 20,
              ),
              TextFormField(
                key: lastNameKey,
                controller: lastNameTextController,
                keyboardType: TextInputType.name,
                decoration: InputDecoration(
                  hintText: "Last Name",
                  labelText: "Last Name",
                  border: OutlineInputBorder(),
                ),
                validator: (value) {
                  if (value == null || value.isEmpty) {
                    return 'Please enter last name';
                  }
                  return null;
                },
              ),
              SizedBox(
                height: 20,
              ),
              TextFormField(
                key: contactNumberKey,
                controller: contactNumberTextController,
                keyboardType: TextInputType.number,
                inputFormatters: [FilteringTextInputFormatter.digitsOnly],
                decoration: InputDecoration(
                  hintText: "Contact Number",
                  labelText: "Contact Number",
                  border: OutlineInputBorder(),
                ),
                validator: (value) {
                  if (value == null || value.isEmpty) {
                    return 'Please enter contact number';
                  } else if (value.length != 10) {
                    return 'Invalid contact number';
                  }
                  return null;
                },
              ),
              SizedBox(
                height: 20,
              ),
              ElevatedButton(
                key: submitKey,
                child: Text("Submit"),
                onPressed: () async {
                  if (_formKey.currentState!.validate()) {
                    addContact();
                    clearTexts();
                    Navigator.pop(context);
                  }
                },
              )
            ],
          ),
        ),
      ),
    );
  }
}

final url = "https://learncontactsapi.herokuapp.com/api/v1/resources/contacts/";

Future<List<Contact>> fetchContacts(http.Client client) async {
  final response = await client.get(Uri.parse(url));
  if (response.statusCode == 200) {
    if (json.decode(response.body).toString().length > 2) {
      List jsonResponse = json.decode(response.body);
      print(jsonResponse.toString());
      return jsonResponse.map((e) => Contact.fromJson(e)).toList();
    } else {
      throw Exception('Failed to load contacts');
    }
  }
  throw Exception('Failed to load contacts');
}

Widget buildContactList({required contacts}) {
  return FutureBuilder<List<Contact>>(
    future: contacts,
    builder: (context, snapshot) {
      if (snapshot.connectionState == ConnectionState.done) {
        if (snapshot.hasData) {
          return ListView.builder(
              itemCount: snapshot.data!.length,
              itemBuilder: (context, index) {
                return Card(
                  margin: EdgeInsets.all(10.0),
                  child: ListTile(
                    leading: CircleAvatar(
                      radius: 40,
                      backgroundImage:
                          AssetImage("assets/icons/img_avatar.png"),
                    ),
                    title: Text(snapshot.data![index].getName()),
                    subtitle: Text(snapshot.data![index].getContactNumber()),
                  ),
                );
              });
        } else if (snapshot.hasError) {
          return Center(
            child: Text(
              'Oops contact load failed !',
              style: TextStyle(
                color: Colors.red,
                fontSize: 20,
              ),
            ),
          );
        }
      }
      return Center(
        child: const CircularProgressIndicator(),
      );
    },
  );
}
