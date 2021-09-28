class Contact {
  final String firstName;
  final String lastName;
  final String contactNumber;

  Contact({
    required this.firstName,
    required this.lastName,
    required this.contactNumber,
  });

  String getName() => firstName + " " + lastName;

  String getFirstName() => firstName;

  String getLastName() => lastName;

  String getContactNumber() => contactNumber;

  factory Contact.fromJson(Map<String, dynamic> json) {
    return Contact(
        firstName: json['first_name'],
        lastName: json['last_name'],
        contactNumber: json['contact_number']);
  }
}
