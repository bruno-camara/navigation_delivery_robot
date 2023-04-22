import time
import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore

cred = credentials.Certificate("./serviceAccountKey.json")
firebase_admin.initialize_app(cred)

db = firestore.client()

robot_state = {
    "in_point_a": 'in-point-a',
    "in_point_b": 'in-point-b',
    "going_to_point_a": 'going-to-point-a',
    "going_to_point_b": 'going-to-point-b',
    "between_a_and_b": 'between-a-and-b',
    "loading": 'loading',
}

current_robot_state = robot_state["loading"]

ref = db.collection(u"robotState").document(u"root")

def set_robot_state(state):
    ref.set({
        'state': state
    })


def go_to_point_a():
    print("Going to point A")

    # emulate reaching point A
    time.sleep(5)

    if current_robot_state == robot_state["going_to_point_a"]:
        print("Reached point A")
        set_robot_state(robot_state["in_point_a"])


def go_to_point_b():
    print("Going to point B")

    # emulate reaching point B
    time.sleep(5)

    if current_robot_state == robot_state["going_to_point_b"]:
        print("Reached point B")
        set_robot_state(robot_state["in_point_b"])


def stop_between_a_and_b():
    print("Stopping between A and B")


def on_robot_state_change(new_state):
    global current_robot_state

    if new_state == current_robot_state:
        return

    if current_robot_state == robot_state["loading"]:
        current_robot_state = new_state
        print("Initializing with state")

    else:
        print("Robot state changed to")
        current_robot_state = new_state

        if new_state == 'going-to-point-a':
            go_to_point_a()

        elif new_state == 'going-to-point-b':
            go_to_point_b()

        elif new_state == 'between-a-and-b':
            stop_between_a_and_b()


def on_snapshot(doc_snapshot, changes, read_time):
    for doc in doc_snapshot:
        if doc.id == 'root':
            new_state = doc.to_dict().get('state')
            on_robot_state_change(new_state)


doc_watch = ref.on_snapshot(on_snapshot)

while True:
    time.sleep(1)
