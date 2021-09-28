from flask import Flask,jsonify,request
from pymongo import MongoClient
import os
import uuid

from pymongo.common import partition_node

app = Flask(__name__)

# MongoDB Config
mongodb_connection_string = os.environ.get('MONGODB_CONNECTION_URL')
mongoClient = MongoClient(mongodb_connection_string,connect=False)
db = mongoClient.contacts

@app.route('/api/v1/resources/contacts/',methods=['POST','GET'])
def get_contacts():
    if request.method == 'GET':
        contacts = []
        results = db.contacts.find()
        for document in results:
            document['_id'] = str(document['_id'])
            contacts.append(document)
        return jsonify(contacts),200

    if request.method == 'POST':
        contact = {
            '_id':uuid.uuid4().hex,
            'first_name':request.get_json().get('first_name'),
            'last_name':request.get_json().get('last_name'),
            'contact_number':request.get_json().get('contact_number')
        }
        print("body json "+request.get_json().get('first_name'))
        if db.contacts.insert_one(contact):
            return  {"status":"sucess"},201
        else:
            return {"status":"failed"}

if __name__ == "__main__":
    app.run(debug=True)