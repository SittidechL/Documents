<!-- https://www.youtube.com/watch?v=BOITPwChVP4&t=1s -->
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Document</title>
    <style>
    label{display: inline-block; width:80px}
    input,select{width:120;}
    </style>
</head>
<body>
  
    <h1 id="bigOne"></h1>
    <label for="Namebox">Namebox</label><input type="text" id="namebox"><br><br>
    <label for="RollNo ">RollNo</label><input type="text" id="rollbox"><br><br>
    <label for="Secbox">Secbox</label><input type="text" id="secbox"><br><br>
    <label for="Secbox">Gender</label><br><br>
    <select id="Genbox">
        <option value="Male">Male</option>
        <option value="Female">Female</option>
    </select><br><br><br>
    <hr>
    <button id="Insbtn">INSERT</button>
    <button id="Selbtn">SELECT</button>
    <button id="Updbtn">UPDATE</button>
    <button id="Delbtn">DELETE</button>


    <!-- The core Firebase JS SDK is always required and must be listed fires  -->
    
    <script src=""></script>

    <script type="module">
        import { initializeApp } from "https://www.gstatic.com/firebasejs/9.1.0/firebase-app.js"
      // REPLACE WITH YOUR web app's Firebase configuration
      var firebaseConfig = {
        apiKey: "AIzaSyD5PtpLJCwz5hb1vLhBVxiUdmFeozyokZ0",
        authDomain: "iot-pex.firebaseapp.com",
        databaseURL: "https://iot-pex-default-rtdb.firebaseio.com",
        projectId: "iot-pex",
        storageBucket: "iot-pex.appspot.com",
        messagingSenderId: "755062941572",
        appId: "1:755062941572:web:3a6a8b87ef568139d49fd1"
      };
      // Initialize Firebase
    //   firebase.initializeApp(firebaseConfig);
    const app = initializeApp(firebaseConfig);

      import  { getDatabase ,ref, set, child, update, remove }
      from "https://www.gstatic.com/firebasejs/9.1.0/firebase-database.js"
    
    const db = getDatabase();

    //   var bigOne= document.getElementById('bigOne');
      var namebox= document.getElementById('Namebox');
      var rollbox= document.getElementById('Rollbox');
      var secbox= document.getElementById('Secbox');
      var genbox= document.getElementById('Genbox');
      
      var insBtn= document.getElementById('Insbtn');
      var selBtn= document.getElementById('Selbtn');
      var updBtn= document.getElementById('Updbtn');
      var delBtn= document.getElementById('Delbtn');
      
    //   var dbRef = firebase.database().ref().child('text');
    //   dbRef.on('value',snap => bigOne.innerText = snap.val());

      function InsertData(){
        set(ref(db, "TheStudents/" + namebox.value),{
            NameOfStd: namebox.value,
            rollbox: rollbox.value,
            Section: secbox.value,
            Gender: genbox.value
        })
        .then(()=>{
            alert("data stored successfully");
        })
        .catch((error)=>{
            alert("unsuccessful, error"+error);
        })
      } 

    //   Assign Events To Butns
      insBtn.addEventListener('click',InsertData);
    </script>
   
</body>
</html>
