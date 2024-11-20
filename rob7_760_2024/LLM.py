import os
from openai import OpenAI

# Assuming the OpenAI API key is set as an environment variable
OPENAI_API_KEY = os.environ['OPENAI_API_KEY']

client = OpenAI(api_key=OPENAI_API_KEY)

while True:
    user_input = input("Enter your command (or type 'exit' to quit): ")
    if user_input.lower() == 'exit':
        print("Exiting the program. Goodbye!")
        break

    response = client.chat.completions.create(
        messages=[
            {
                "role": "system",
                "content": 'You are my semantic processing assistant. When you recieve a string, you should parse it down to the locations included. Like "Go to the bed behind the cabin" then you should return [bed, cabin], "Go get my medicine on the counter behind the sofa" then you should return [sofa, counter, medicine]. If a plural is present, return the item the appropriate amount of times. Like "Go get the remote by the chairs" should return [chair, chair, remote].'
            },
            {"role": "user", "content": user_input},
        ],
        model="gpt-4o-mini",
    )

    # Accessing the content directly as an attribute
    parsed_locations = response.choices[0].message.content
    print(f"Parsed locations: {parsed_locations}")
