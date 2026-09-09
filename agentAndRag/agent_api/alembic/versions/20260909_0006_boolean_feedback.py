"""Store message feedback as one nullable boolean, preserving existing labels."""
import sqlalchemy as sa
from alembic import op

revision = "20260909_0006"
down_revision = "20260905_0005"
branch_labels = None
depends_on = None


def upgrade() -> None:
    connection = op.get_bind()
    invalid = connection.scalar(sa.text(
        "SELECT COUNT(*) FROM platform_messages WHERE feedback_rating IS NOT NULL "
        "AND feedback_rating NOT IN ('up', 'down')"
    ))
    if invalid:
        raise ValueError("invalid historical feedback_rating; migration aborted")
    columns = {column["name"] for column in sa.inspect(connection).get_columns("platform_messages")}
    # Revision 0001 creates current metadata on a fresh database. Never overwrite
    # existing boolean feedback if an unexpected mixed schema contains data.
    if "feedback_is_good" in columns:
        if connection.scalar(sa.text("SELECT COUNT(*) FROM platform_messages WHERE feedback_is_good IS NOT NULL")):
            raise ValueError("mixed feedback schema contains boolean data; migration aborted")
    else:
        op.add_column("platform_messages", sa.Column("feedback_is_good", sa.Boolean(), nullable=True))
    connection.execute(sa.text(
        "UPDATE platform_messages SET feedback_is_good = CASE "
        "WHEN feedback_rating = 'up' THEN TRUE WHEN feedback_rating = 'down' THEN FALSE ELSE NULL END"
    ))
    with op.batch_alter_table("platform_messages") as batch:
        batch.drop_column("feedback_rating")


def downgrade() -> None:
    op.add_column("platform_messages", sa.Column("feedback_rating", sa.String(8), nullable=True))
    op.get_bind().execute(sa.text(
        "UPDATE platform_messages SET feedback_rating = CASE "
        "WHEN feedback_is_good IS TRUE THEN 'up' WHEN feedback_is_good IS FALSE THEN 'down' ELSE NULL END"
    ))
    with op.batch_alter_table("platform_messages") as batch:
        batch.drop_column("feedback_is_good")
