from twilio.rest import Client

account_sid = '***'
auth_token = '***'
client = Client(account_sid, auth_token)

message = client.messages.create(
    messaging_service_sid='****',
  body='http://192.168.0.23:8000/임수빈_qr_code.png 방문 시 QR을 제시해주세요.',
  to='+***'
)

print(message.sid)