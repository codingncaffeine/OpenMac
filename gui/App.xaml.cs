using System.Windows;
using System.Windows.Threading;

namespace OpenMac.Gui;

public partial class App : Application
{
    protected override void OnStartup(StartupEventArgs e)
    {
        Log.Init();
        Log.Line("app startup");

        DispatcherUnhandledException += OnDispatcherUnhandled;
        AppDomain.CurrentDomain.UnhandledException += (_, ev) =>
            Log.Line("FATAL (AppDomain): " + ev.ExceptionObject);

        base.OnStartup(e);
    }

    protected override void OnExit(ExitEventArgs e)
    {
        Log.Line("app exit");
        base.OnExit(e);
    }

    private void OnDispatcherUnhandled(object sender, DispatcherUnhandledExceptionEventArgs e)
    {
        Log.Line("UNHANDLED: " + e.Exception);
        MessageBox.Show(
            "An unexpected error occurred:\n\n" + e.Exception.Message + "\n\nDetails were written to:\n" + Log.Path,
            "OpenMac", MessageBoxButton.OK, MessageBoxImage.Error);
        e.Handled = true;   // keep the app alive if we can
    }
}
