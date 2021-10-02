import unittest
import json
from app import app,db
class ContactAPITest(unittest.TestCase):
    def setUp(self) -> None:
        self.app = app.test_client()
        self.db = db
    
    def test_get_contacts(self):
        response = self.app.get('/api/v1/resources/contacts/')
        self.assertEqual(200,response.status_code)

    def test_post_contact(self):
        contact = json.dumps({
            "_id":"dc31c5c85a0e4aff9482c082058a7881",
            "first_name":"Shrinath",
            "last_name":"Gupta",
            "contact_number":"8433530825"})
        header = {"Content-Type": "application/json"}       
        response = self.app.post('/api/v1/resources/contacts/',headers=header,data=contact)
        self.assertEqual(201,response.status_code) 



