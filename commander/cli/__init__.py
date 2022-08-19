from pathlib import Path

import click


from commander.simexp_base import SimulationExperimentCommand, simexp_command
from commander.log import setup_logging




@click.group()
@click.pass_context
@click.option(
    "-o",
    "--output-dir",
    type=click.Path(file_okay=False, path_type=Path, writable=True, readable=True),
    default=Path("./output_data"),
)
@click.option(
    "-v",
    "--verbose",
    is_flag=True,
    default=False
)
def cli(ctx: click.Context, output_dir: Path, verbose: bool) -> None:
    ctx.ensure_object(dict)
    ctx.obj["output_dir"] = output_dir
    ctx.obj["verbose"] = verbose

    global_ctx = click.get_current_context()

    assert global_ctx.invoked_subcommand
    setup_logging(command=global_ctx.invoked_subcommand, debug=verbose)


cli.add_command(simexp_command(SimulationExperimentCommand.EXPERIMENT))


def run() -> None:
    cli(obj={})


if __name__ == "__main__":
    run()
