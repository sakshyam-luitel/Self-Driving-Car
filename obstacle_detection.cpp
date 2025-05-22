// Define motor pins
const int m11 = 16;
const int m12 = 12;
const int m21 = 21;
const int m22 = 20;

// Define ultrasonic sensor pins
const int TRIG = 17;
const int ECHO = 27;

// Define the LED pin
const int led = 22;

// Initialize variables
long duration;
int distance;
int avgDistance = 0;
int count = 0;
int flag = 0;

void setup()
{
    // Set motor pins as outputs
    pinMode(m11, OUTPUT);
    pinMode(m12, OUTPUT);
    pinMode(m21, OUTPUT);
    pinMode(m22, OUTPUT);

    // Set ultrasonic sensor pins
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);

    // Set LED pin as output
    pinMode(led, OUTPUT);

    // Start serial communication
    Serial.begin(9600);

    // Turn on the LED to indicate the system is ready
    digitalWrite(led, HIGH);

    delay(5000); // Wait for 5 seconds before starting
}

void loop()
{
    avgDistance = 0;

    // Take 5 distance readings and average them
    for (int i = 0; i < 5; i++)
    {
        // Send a pulse to trigger the ultrasonic sensor
        digitalWrite(TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG, LOW);

        // Measure the pulse duration
        duration = pulseIn(ECHO, HIGH);

        // Calculate the distance
        distance = duration * 0.0344 / 2; // Speed of sound is 0.0344 cm/us
        avgDistance += distance;

        delay(50); // Small delay between readings
    }

    avgDistance = avgDistance / 5; // Average the distance readings

    Serial.print("Distance: ");
    Serial.println(avgDistance);

    if (avgDistance < 25)
    {
        // If distance is less than 25 cm, stop and avoid obstacle
        count++;
        stop();
        delay(1500); // Stop for 1.5 seconds
        if (count % 3 == 1 && flag == 0)
        {
            right();
            flag = 1;
        }
        else
        {
            left();
            flag = 0;
        }
        delay(1500); // Turn for 1.5 seconds
        stop();
        delay(1000); // Wait for 1 second
    }
    else
    {
        forward(); // Move forward if no obstacle
        flag = 0;
    }
}

void stop()
{
    // Stop the motors
    digitalWrite(m11, LOW);
    digitalWrite(m12, LOW);
    digitalWrite(m21, LOW);
    digitalWrite(m22, LOW);
    Serial.println("Stop");
}

void forward()
{
    // Move forward
    digitalWrite(m11, LOW);
    digitalWrite(m12, HIGH);
    digitalWrite(m21, HIGH);
    digitalWrite(m22, LOW);
    Serial.println("Forward");
}

void back()
{
    // Move backward
    digitalWrite(m11, LOW);
    digitalWrite(m12, HIGH);
    digitalWrite(m21, LOW);
    digitalWrite(m22, HIGH);
    Serial.println("Back");
}

void left()
{
    // Turn left
    digitalWrite(m11, LOW);
    digitalWrite(m12, LOW);
    digitalWrite(m21, HIGH);
    digitalWrite(m22, LOW);
    Serial.println("Left");
}

void right()
{
    // Turn right
    digitalWrite(m11, LOW);
    digitalWrite(m12, HIGH);
    digitalWrite(m21, LOW);
    digitalWrite(m22, LOW);
    Serial.println("Right");
}
