//
//  benjominiOSApp.swift
//  benjominiOS
//
//  Created by k19086kk on 2021/10/17.
//

import SwiftUI
import Firebase

@main
struct benjominiOSApp: App {
    @UIApplicationDelegateAdaptor(AppDelegate.self) var appDelegate // 追加
    var body: some Scene {
        WindowGroup {
            ContentView()
        }
    }
}
class AppDelegate: UIResponder, UIApplicationDelegate {
    func application(_ application: UIApplication, didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey : Any]? = nil) -> Bool {
        FirebaseApp.configure()
        return true
    }
}
