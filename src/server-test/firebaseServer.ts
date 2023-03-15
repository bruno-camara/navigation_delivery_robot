import { cert, initializeApp } from 'firebase-admin/app';
import { getFirestore } from 'firebase-admin/firestore';
import fs from 'fs';
import url from 'url';

function getFilePath(relative: string) {
  return url.fileURLToPath(new URL(relative, import.meta.url));
}

initializeApp({
  credential: cert(getFilePath('./serviceAccountKey.json')),
});

const db = getFirestore();

const ref = db.collection('robotState').doc('root');

const roboState = [
  'in-point-a',
  'in-point-b',
  'going-to-point-a',
  'going-to-point-b',
  'between-a-and-b',
  'loading',
];

ref.onSnapshot((doc) => {
  const newState = doc.data()?.state.toString();

  console.log('New state from server: ', newState);

  fs.writeFileSync(getFilePath('./from-server.txt'), newState);
});

fs.watchFile(getFilePath('./from-robot.txt'), () => {
  const newState = fs.readFileSync(getFilePath('./from-robot.txt'), 'utf8');

  console.log('Set state from robot: ', newState);

  if (roboState.includes(newState)) {
    ref.set({ state: newState });
  } else {
    console.log('Invalid state');
  }
});
