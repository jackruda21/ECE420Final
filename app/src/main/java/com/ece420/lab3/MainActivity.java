package com.ece420.lab3;

import android.annotation.SuppressLint;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.content.pm.ActivityInfo;
import android.Manifest;
import android.os.Bundle;
import android.support.v4.content.ContextCompat;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;


public class MainActivity extends AppCompatActivity {

    // Flag to control app behavior
    public static int appFlag = 0;
    // UI Variables
    private Button subButton;
    private Button gateButton;
    private Button weinerButton;

    @SuppressLint("MissingInflatedId")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.main_activity);
        super.setRequestedOrientation (ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);

        // Request User Permission to RECORD_AUDIO
        if(ContextCompat.checkSelfPermission(this, Manifest.permission.RECORD_AUDIO) == PackageManager.PERMISSION_DENIED){
            ActivityCompat.requestPermissions(this, new String[] {Manifest.permission.RECORD_AUDIO}, 1);}

        // Setup Button for Spectral Subtraction
        subButton = (Button) findViewById(R.id.specSub);
        subButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                appFlag = 1;
                startActivity(new Intent(MainActivity.this, AudioActivity.class));
            }
        });

        // Setup Button for Spectral Gating
        gateButton = (Button) findViewById(R.id.specGate);
        gateButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                appFlag = 2;
                startActivity(new Intent(MainActivity.this, AudioActivity.class));
            }
        });

        // Setup Button for Wiener Filter
        weinerButton = (Button) findViewById(R.id.wiener);
        weinerButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                appFlag = 3;
                startActivity(new Intent(MainActivity.this, AudioActivity.class));
            }
        });

    }

    @Override
    protected void onResume(){
        super.setRequestedOrientation (ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
        super.onResume();
    }

}
